//
// Created by victor on 16.11.18.
//

#include "voxgraph/mapper/voxgraph_mapper.h"
#include <minkindr_conversions/kindr_xml.h>
#include <visualization_msgs/MarkerArray.h>
#include <voxblox_ros/ros_params.h>
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
      submap_collection_(submap_config_),
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
  XmlRpc::XmlRpcValue T_odom__sensor_xml;
  CHECK(nh_private_.getParam("T_odom_sensor", T_odom__sensor_xml))
      << "The transform from the odom frame to the sensor frame "
      << "T_odom__sensor is required but was not set.";
  kindr::minimal::xmlRpcToKindr(T_odom__sensor_xml, &T_odom__sensor_);

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
  submap_vis_.publishSeparatedMesh(submap_collection_, world_frame_,
                                   separated_mesh_pub_);
  return true;  // Tell ROS it succeeded
}

bool VoxgraphMapper::publishCombinedMeshCallback(
    std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {
  submap_vis_.publishCombinedMesh(submap_collection_, world_frame_,
                                  combined_mesh_pub_);
  return true;  // Tell ROS it succeeded
}

bool VoxgraphMapper::saveToFileCallback(
    voxblox_msgs::FilePath::Request &request,
    voxblox_msgs::FilePath::Response &response) {
  submap_collection_.saveToFile(request.file_path);
  return true;
}

void VoxgraphMapper::pointcloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr &pointcloud_msg) {
  // Lookup the robot pose at the time of the pointcloud message
  updateToOdomAt(pointcloud_msg->header.stamp);

  // Check if it's time to create a new submap
  if (shouldCreateNewSubmap(pointcloud_msg->header.stamp)) {
    // Define the new submap frame to be at the current robot pose
    // and have its Z-axis aligned with gravity
    voxblox::Transformation::Vector6 T_vec = T_world__odom_.log();
    T_vec[3] = 0;  // Set roll to zero
    T_vec[4] = 0;  // Set pitch to zero
    voxblox::Transformation T_world__new_submap =
        voxblox::Transformation::exp(T_vec);

    // Create the new submap
    submap_collection_.createNewSubMap(T_world__new_submap);
    current_submap_creation_stamp_ = pointcloud_msg->header.stamp;
    ROS_INFO_STREAM("Creating submap nr: " << submap_collection_.size()
                                           << " with pose\n"
                                           << T_world__new_submap);

    // Update the TSDF submap collection visualization in Rviz
    submap_vis_.publishSeparatedMesh(submap_collection_, world_frame_,
                                     separated_mesh_pub_);
    if (debug_) {
      TfHelper::publishTransform(T_world__new_submap, "world",
                                 "debug_T_world__new_submap", true,
                                 pointcloud_msg->header.stamp);
    }
  }

  // Lookup the sensor's pose in world frame
  voxblox::Transformation T_world__sensor;
  if (use_gt_ptcloud_pose_from_sensor_tf_) {
    ROS_WARN_STREAM_THROTTLE(20, "Using ground truth pointcloud poses"
                                     << " provided by TF from " << world_frame_
                                     << " to "
                                     << pointcloud_msg->header.frame_id);
    transformer_.lookupTransform(pointcloud_msg->header.frame_id, world_frame_,
                                 pointcloud_msg->header.stamp,
                                 &T_world__sensor);
    if (debug_) {
      TfHelper::publishTransform(T_world__sensor, "world",
                                 "debug_T_world__sensor", false,
                                 pointcloud_msg->header.stamp);
    }
  } else {
    // Get the pose of the sensor in world frame
    T_world__sensor = T_world__odom_ * T_odom__sensor_;
    if (debug_) {
      TfHelper::publishTransform(T_world__odom_, "world", "debug_T_world__odom",
                                 false, pointcloud_msg->header.stamp);
      TfHelper::publishTransform(T_odom__sensor_, "world",
                                 "debug_T_odom__sensor", false,
                                 pointcloud_msg->header.stamp);
      TfHelper::publishTransform(T_world__sensor, "world",
                                 "debug_T_world__sensor", false,
                                 pointcloud_msg->header.stamp);
    }
  }

  integratePointcloud(pointcloud_msg, T_world__sensor);
}

void VoxgraphMapper::updateToOdomAt(ros::Time timestamp) {
  voxblox::Transformation transform_getter;
  double t_waited = 0;  // Total time spent waiting for the updated pose
  double t_max = 0.08;  // Maximum time to wait before giving up
  const ros::Duration timeout(0.005);  // Timeout between each update attempt
  while (t_waited < t_max) {
    if (transformer_.lookupTransform(odom_tf_frame_, world_frame_, timestamp,
                                     &transform_getter)) {
      T_world__odom_ = transform_getter;
      break;
    }
    timeout.sleep();
    t_waited += timeout.toSec();
  }
  if (t_waited >= t_max) {
    ROS_WARN("Waited %.3fs, but still couldn't update T_world__odom", t_waited);
  }
}

bool VoxgraphMapper::shouldCreateNewSubmap(const ros::Time &current_time) {
  ros::Time new_submap_creation_deadline =
      current_submap_creation_stamp_ + submap_creation_interval_;
  // TODO(victorr): Also take the pose uncertainty etc into account
  return current_time > new_submap_creation_deadline;
}

void VoxgraphMapper::integratePointcloud(
    const sensor_msgs::PointCloud2::ConstPtr &pointcloud_msg,
    const voxblox::Transformation &T_world__sensor) {
  // Transform the sensor pose into the submap frame
  const cblox::Transformation T_world__submap =
      submap_collection_.getActiveSubMapPose();
  const cblox::Transformation T_submap__sensor =
      T_world__submap.inverse() * T_world__sensor;

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
        submap_collection_.getActiveTsdfMapPtr()->getTsdfLayerPtr()));
    ROS_INFO("Initialized TSDF Integrator");
  }

  // TODO(victorr): Implement Cartographer style simultaneous integration into
  //                multiple submaps for guaranteed overlap

  // Point the integrator to the current submap
  tsdf_integrator_->setLayer(
      submap_collection_.getActiveTsdfMapPtr()->getTsdfLayerPtr());

  // Integrate the pointcloud (and report timings if requested)
  ROS_INFO_COND(verbose_, "Integrating a pointcloud with %lu points.",
                pointcloud.size());
  ros::WallTime start = ros::WallTime::now();
  tsdf_integrator_->integratePointCloud(T_submap__sensor, pointcloud, colors);
  ros::WallTime end = ros::WallTime::now();
  ROS_INFO_COND(
      verbose_,
      "Finished integrating in %f seconds, submap %u now has %lu blocks.",
      (end - start).toSec(), submap_collection_.getActiveSubMapID(),
      submap_collection_.getActiveTsdfMap()
          .getTsdfLayer()
          .getNumberOfAllocatedBlocks());
}
}  // namespace voxgraph
