//
// Created by victor on 16.11.18.
//

#include "voxgraph/mapper/voxgraph_mapper.h"
#include <minkindr_conversions/kindr_xml.h>
#include <std_srvs/SetBool.h>
#include <visualization_msgs/MarkerArray.h>
#include <voxblox_ros/ros_params.h>
#include <memory>
#include <string>
#include "voxgraph/submap_registration/submap_registerer.h"
#include "voxgraph/utils/tf_helper.h"

namespace voxgraph {
VoxgraphMapper::VoxgraphMapper(const ros::NodeHandle &nh,
                               const ros::NodeHandle &nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      verbose_(false),
      debug_(false),
      subscriber_queue_length_(100),
      pointcloud_topic_("pointcloud"),
      transformer_(nh, nh_private),
      odom_tf_frame_("odom"),
      world_frame_("world"),
      use_gt_ptcloud_pose_from_sensor_tf_(false),
      submap_collection_(std::make_shared<SubmapCollection>(submap_config_)),
      pose_graph_(submap_collection_),
      submap_vis_(submap_config_),
      submap_creation_interval_(20),  // In seconds
      current_submap_creation_stamp_(0) {
  // Setup interaction with ROS
  getParametersFromRos();
  subscribeToTopics();
  advertiseTopics();
  advertiseServices();
}

void VoxgraphMapper::getParametersFromRos() {
  nh_private_.param("verbose", verbose_, verbose_);
  nh_private_.param("debug", debug_, debug_);
  nh_private_.param("subscriber_queue_length", subscriber_queue_length_,
                    subscriber_queue_length_);
  nh_private_.param("pointcloud_topic", pointcloud_topic_, pointcloud_topic_);
  nh_private_.param("odom_tf_frame", odom_tf_frame_, odom_tf_frame_);
  nh_private_.param("world_frame", world_frame_, world_frame_);
  nh_private_.param("use_gt_ptcloud_pose_from_sensor_tf",
                    use_gt_ptcloud_pose_from_sensor_tf_,
                    use_gt_ptcloud_pose_from_sensor_tf_);

  // Get the submap creation interval as a ros::Duration
  double submap_creation_interval_temp;
  if (nh_private_.getParam("submap_creation_interval",
                           submap_creation_interval_temp)) {
    submap_creation_interval_ = ros::Duration(submap_creation_interval_temp);
  }

  // Load the transform from the odometry frame to the sensor frame
  XmlRpc::XmlRpcValue T_odom_sensor_xml;
  CHECK(nh_private_.getParam("T_odom_sensor", T_odom_sensor_xml))
      << "The transform from the odom frame to the sensor frame "
      << "T_odom_sensor is required but was not set.";
  kindr::minimal::xmlRpcToKindr(T_odom_sensor_xml, &T_robot_sensor_);

  // Read TSDF integrator params from ROS (stored in their own sub-namespace)
  ros::NodeHandle nh_tsdf_integrator(nh_private_, "tsdf_integrator");
  tsdf_integrator_config_ =
      voxblox::getTsdfIntegratorConfigFromRosParam(nh_tsdf_integrator);
}

void VoxgraphMapper::subscribeToTopics() {
  pointcloud_subscriber_ =
      nh_.subscribe(pointcloud_topic_, subscriber_queue_length_,
                    &VoxgraphMapper::pointcloudCallback, this);
}

void VoxgraphMapper::advertiseTopics() {
  separated_mesh_pub_ = nh_private_.advertise<visualization_msgs::Marker>(
      "separated_mesh", subscriber_queue_length_);
  combined_mesh_pub_ = nh_private_.advertise<visualization_msgs::Marker>(
      "combined_mesh", subscriber_queue_length_);
}

void VoxgraphMapper::advertiseServices() {
  publish_separated_mesh_srv_ = nh_private_.advertiseService(
      "publish_separated_mesh", &VoxgraphMapper::publishSeparatedMeshCallback,
      this);
  publish_combined_mesh_srv_ = nh_private_.advertiseService(
      "publish_combined_mesh", &VoxgraphMapper::publishCombinedMeshCallback,
      this);
  save_to_file_srv_ = nh_private_.advertiseService(
      "save_to_file", &VoxgraphMapper::saveToFileCallback, this);
}

bool VoxgraphMapper::publishSeparatedMeshCallback(
    std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {
  submap_vis_.publishSeparatedMesh(*submap_collection_, world_frame_,
                                   separated_mesh_pub_);
  return true;  // Tell ROS it succeeded
}

bool VoxgraphMapper::publishCombinedMeshCallback(
    std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {
  submap_vis_.publishCombinedMesh(*submap_collection_, world_frame_,
                                  combined_mesh_pub_);
  return true;  // Tell ROS it succeeded
}

bool VoxgraphMapper::saveToFileCallback(
    voxblox_msgs::FilePath::Request &request,
    voxblox_msgs::FilePath::Response &response) {
  submap_collection_->saveToFile(request.file_path);
  return true;
}

// TODO(victorr): Check if this method can be broken down in clearer submethods
void VoxgraphMapper::pointcloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr &pointcloud_msg) {
  // Lookup the robot pose at the time of the pointcloud message
  Transformation T_world_robot;
  if (!lookup_T_world_robot(pointcloud_msg->header.stamp, &T_world_robot)) {
    // If the pose cannot be found, the pointcloud is skipped
    ROS_WARN_STREAM("Skipping pointcloud since the robot pose at time "
                    << pointcloud_msg->header.stamp << " is unknown.");
    return;
  }

  // Check if it's time to create a new submap
  if (shouldCreateNewSubmap(pointcloud_msg->header.stamp)) {
    // Add the finished submap to the pose graph,
    // unless the first submap has not yet been created
    if (!submap_collection_->empty()) {
      SubmapID finished_submap_id = submap_collection_->getActiveSubMapID();
      // TODO(victorr): Move the graph optimization and visuals in
      //                separate threads instead of pausing the bag
      static ros::ServiceClient rosbag_pause_srv =
          nh_.serviceClient<std_srvs::SetBool>("/player/pause_playback");
      std_srvs::SetBool srv_msg;
      srv_msg.request.data = true;
      rosbag_pause_srv.call(srv_msg);

      // Generate the finished submap's ESDF
      submap_collection_->generateEsdfById(finished_submap_id);

      // Configure the submap node and add it to the pose graph
      SubmapNode::Config node_config;
      node_config.submap_id = finished_submap_id;
      CHECK(submap_collection_->getSubMapPose(
          finished_submap_id, &node_config.initial_submap_pose));
      if (finished_submap_id == 0) {
        std::cout << "Setting pose of submap 0 to constant" << std::endl;
        node_config.set_constant = true;
      } else {
        node_config.set_constant = false;
      }
      pose_graph_.addNode(node_config);

      // Constrain the finished submap to all submaps it overlaps with
      const VoxgraphSubmap &finished_submap =
          submap_collection_->getSubMap(finished_submap_id);
      for (const auto &other_submap_ptr : submap_collection_->getSubMaps()) {
        if (other_submap_ptr->getID() != finished_submap_id) {
          if (finished_submap.overlapsWith(*other_submap_ptr)) {
            // Add the constraint
            RegistrationConstraint::Config constraint_config = {
                finished_submap_id, other_submap_ptr->getID()};
            pose_graph_.addConstraint(constraint_config);
          }
        }
      }

      // Optimize the pose graph
      ROS_INFO("Optimizing the pose graph");
      pose_graph_.optimize();

      // Remember the pose of the current submap (later used to eliminate drift)
      Transformation T_W_S_old = submap_collection_->getActiveSubMapPose();

      // Update the submap poses
      for (const auto &submap_pose_kv : pose_graph_.getSubmapPoses()) {
        submap_collection_->setSubMapPose(submap_pose_kv.first,
                                          submap_pose_kv.second);
      }

      // Update the fictional odometry origin to cancel out drift
      Transformation T_W_S_new = submap_collection_->getActiveSubMapPose();
      T_W_D_ = T_W_S_new * T_W_S_old.inverse() * T_W_D_;
      T_world_robot = T_W_S_new * T_W_S_old.inverse() * T_world_robot;

      // Update the submap collection visualization in Rviz
      submap_vis_.publishSeparatedMesh(*submap_collection_, world_frame_,
                                       separated_mesh_pub_);

      // TODO(victorr): Move the graph optimization and visuals in
      //                separate threads instead of pausing the bag
      srv_msg.request.data = false;
      rosbag_pause_srv.call(srv_msg);
    }

    // Define the new submap frame to be at the current robot pose
    // and have its Z-axis aligned with gravity
    Transformation::Vector6 T_vec = T_world_robot.log();
    T_vec[3] = 0;  // Set roll to zero
    T_vec[4] = 0;  // Set pitch to zero
    Transformation T_world__new_submap = Transformation::exp(T_vec);
    if (debug_) {
      TfHelper::publishTransform(T_world__new_submap, "world",
                                 "debug_T_world__new_submap", true,
                                 pointcloud_msg->header.stamp);
    }

    // Create the new submap
    SubmapID new_submap_id =
        submap_collection_->createNewSubMap(T_world__new_submap);
    current_submap_creation_stamp_ = pointcloud_msg->header.stamp;
    ROS_INFO_STREAM("Creating submap: " << new_submap_id << " with pose\n"
                                        << T_world__new_submap);
  }

  // Lookup the sensor's pose in world frame
  Transformation T_world_sensor;
  if (use_gt_ptcloud_pose_from_sensor_tf_) {
    ROS_WARN_STREAM_THROTTLE(20, "Using ground truth pointcloud poses"
                                     << " provided by TF from " << world_frame_
                                     << " to "
                                     << pointcloud_msg->header.frame_id);
    transformer_.lookupTransform(pointcloud_msg->header.frame_id, world_frame_,
                                 pointcloud_msg->header.stamp, &T_world_sensor);
    if (debug_) {
      TfHelper::publishTransform(T_world_sensor, "world",
                                 "debug_T_world_sensor", false,
                                 pointcloud_msg->header.stamp);
    }
  } else {
    // Get the pose of the sensor in world frame
    T_world_sensor = T_world_robot * T_robot_sensor_;
    if (debug_) {
      // TODO(victorr): Update these visuals to show the appropriate transforms
      TfHelper::publishTransform(T_world_robot, "world", "debug_T_world_robot",
                                 false, pointcloud_msg->header.stamp);
      TfHelper::publishTransform(T_robot_sensor_, "world",
                                 "debug_T_robot_sensor", false,
                                 pointcloud_msg->header.stamp);
      TfHelper::publishTransform(T_world_sensor, "world",
                                 "debug_T_world_sensor", false,
                                 pointcloud_msg->header.stamp);
    }
  }

  integratePointcloud(pointcloud_msg, T_world_sensor);
}

bool VoxgraphMapper::lookup_T_world_robot(ros::Time timestamp,
                                          Transformation *T_world_robot) {
  CHECK_NOTNULL(T_world_robot);
  Transformation T_D_R;  // Transform corresponding to the received odometry TF
  double t_waited = 0;   // Total time spent waiting for the updated pose
  double t_max = 0.08;   // Maximum time to wait before giving up
  const ros::Duration timeout(0.005);  // Timeout between each update attempt
  while (t_waited < t_max) {
    if (transformer_.lookupTransform(odom_tf_frame_, world_frame_, timestamp,
                                     &T_D_R)) {
      *T_world_robot = T_W_D_ * T_D_R;
      return true;
    }
    timeout.sleep();
    t_waited += timeout.toSec();
  }
  ROS_WARN("Waited %.3fs, but still could not get the updated odom", t_waited);
  return false;
}

bool VoxgraphMapper::shouldCreateNewSubmap(const ros::Time &current_time) {
  ros::Time new_submap_creation_deadline =
      current_submap_creation_stamp_ + submap_creation_interval_;
  // TODO(victorr): Also take the pose uncertainty etc into account
  return current_time > new_submap_creation_deadline;
}

void VoxgraphMapper::integratePointcloud(
    const sensor_msgs::PointCloud2::ConstPtr &pointcloud_msg,
    const Transformation &T_world_sensor) {
  // Transform the sensor pose into the submap frame
  const Transformation T_world_submap =
      submap_collection_->getActiveSubMapPose();
  const Transformation T_submap_sensor =
      T_world_submap.inverse() * T_world_sensor;

  // Convert pointcloud_msg into voxblox::Pointcloud
  pcl::PointCloud<pcl::PointXYZI> pointcloud_pcl;
  pcl::fromROSMsg(*pointcloud_msg, pointcloud_pcl);
  voxblox::Pointcloud pointcloud;
  voxblox::Colors colors;
  pointcloud.reserve(pointcloud_pcl.size());
  colors.reserve(pointcloud_pcl.size());

  // Filter out NaNs while compiling the voxblox pointcloud
  for (const auto &point : pointcloud_pcl.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }

    // Store the point's coordinates
    pointcloud.push_back(voxblox::Point(point.x, point.y, point.z));

    // Map the intensity to a color
    // NOTE: The intensity is a float whose maximum value is sensor dependent.
    //       A scaled 'fast sigmoid' function is therefore used to smoothly map
    //       it's [-Inf, Inf] range to the blue channel with range [0, 255].
    //       Note that other mappings might be better suited, especially when
    //       using a sensor whose intensity is guaranteed to be positive.
    //       This is just quick solution.
    double I = point.intensity;
    I = I / (1 + std::abs(I)) * 127.5 + 127.5;
    auto blue = static_cast<uint8_t>(I);
    colors.push_back(voxblox::Color(0, 0, blue, 1));
  }

  // Initialize the TSDF integrator if this has not yet been done
  if (!tsdf_integrator_) {
    tsdf_integrator_.reset(new voxblox::FastTsdfIntegrator(
        tsdf_integrator_config_,
        submap_collection_->getActiveTsdfMapPtr()->getTsdfLayerPtr()));
    ROS_INFO("Initialized TSDF Integrator");
  }

  // TODO(victorr): Implement Cartographer style simultaneous integration into
  //                multiple submaps for guaranteed overlap

  // Point the integrator to the current submap
  tsdf_integrator_->setLayer(
      submap_collection_->getActiveTsdfMapPtr()->getTsdfLayerPtr());

  // Integrate the pointcloud (and report timings if requested)
  ROS_INFO_COND(verbose_, "Integrating a pointcloud with %lu points.",
                pointcloud.size());
  ros::WallTime start = ros::WallTime::now();
  tsdf_integrator_->integratePointCloud(T_submap_sensor, pointcloud, colors);
  ros::WallTime end = ros::WallTime::now();
  ROS_INFO_COND(
      verbose_,
      "Finished integrating in %f seconds, submap %u now has %lu blocks.",
      (end - start).toSec(), submap_collection_->getActiveSubMapID(),
      submap_collection_->getActiveTsdfMap()
          .getTsdfLayer()
          .getNumberOfAllocatedBlocks());
}
}  // namespace voxgraph
