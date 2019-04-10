//
// Created by victor on 16.11.18.
//

#include "voxgraph/frontend/voxgraph_mapper.h"
#include <minkindr_conversions/kindr_xml.h>
#include <visualization_msgs/MarkerArray.h>
#include <memory>
#include <string>
#include "voxgraph/backend/constraint/cost_functions/submap_registration/submap_registerer.h"
#include "voxgraph/frontend/submap_collection/submap_timeline.h"
#include "voxgraph/tools/tf_helper.h"

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
      submap_collection_ptr_(
          std::make_shared<VoxgraphSubmapCollection>(submap_config_)),
      pose_graph_interface_(submap_collection_ptr_),
      pointcloud_processor_(submap_collection_ptr_),
      submap_vis_(submap_config_),
      rosbag_helper_(nh) {
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
  double interval_temp;
  if (nh_private_.getParam("submap_creation_interval", interval_temp)) {
    submap_collection_ptr_->setSubmapCreationInterval(
        ros::Duration(interval_temp));
  }

  // Load the transform from the odometry frame to the sensor frame
  XmlRpc::XmlRpcValue T_odom_sensor_xml;
  CHECK(nh_private_.getParam("T_odom_sensor", T_odom_sensor_xml))
      << "The transform from the odom frame to the sensor frame "
      << "T_odom_sensor is required but was not set.";
  kindr::minimal::xmlRpcToKindr(T_odom_sensor_xml, &T_robot_sensor_);

  // Read TSDF integrator params from ROS (stored in their own sub-namespace)
  ros::NodeHandle nh_tsdf_integrator(nh_private_, "tsdf_integrator");
  pointcloud_processor_.setTsdfIntegratorConfigFromRosParam(nh_tsdf_integrator);
  //  tsdf_integrator_config_ =
  //      voxblox::getTsdfIntegratorConfigFromRosParam(nh_tsdf_integrator);
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
  submap_vis_.publishSeparatedMesh(*submap_collection_ptr_, world_frame_,
                                   separated_mesh_pub_);
  return true;  // Tell ROS it succeeded
}

bool VoxgraphMapper::publishCombinedMeshCallback(
    std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {
  submap_vis_.publishCombinedMesh(*submap_collection_ptr_, world_frame_,
                                  combined_mesh_pub_);
  return true;  // Tell ROS it succeeded
}

bool VoxgraphMapper::saveToFileCallback(
    voxblox_msgs::FilePath::Request &request,
    voxblox_msgs::FilePath::Response &response) {
  submap_collection_ptr_->saveToFile(request.file_path);
  return true;
}

void VoxgraphMapper::pointcloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr &pointcloud_msg) {
  // Lookup the robot pose at the time of the pointcloud message
  ros::Time current_timestamp = pointcloud_msg->header.stamp;
  Transformation T_world_robot;
  if (!lookup_T_world_robot(current_timestamp, &T_world_robot)) {
    // If the pose cannot be found, the pointcloud is skipped
    ROS_WARN_STREAM("Skipping pointcloud since the robot pose at time "
                    << current_timestamp << " is unknown.");
    return;
  }

  // Check if it's time to create a new submap
  if (submap_collection_ptr_->shouldCreateNewSubmap(current_timestamp)) {
    // Add the finished submap to the pose graph, optimize and correct for drift
    // NOTE: We only add submaps to the pose graph once they're finished to
    //       avoid generating their ESDF more than once
    if (!submap_collection_ptr_->empty()) {
      // Pause the rosbag (remove this once the system runs in real-time)
      rosbag_helper_.pauseRosbag();

      // Add the finished submap to the pose graph, including an odometry link
      SubmapID finished_submap_id = submap_collection_ptr_->getActiveSubMapID();
      pose_graph_interface_.addSubmap(finished_submap_id, true);

      // Optimize the pose graph
      ROS_INFO("Optimizing the pose graph");
      pose_graph_interface_.optimize();

      // Remember the pose of the current submap (used later to eliminate drift)
      Transformation T_W_S_old = submap_collection_ptr_->getActiveSubMapPose();

      // Update the submap poses
      pose_graph_interface_.updateSubmapCollectionPoses();

      // Update the fictional odometry origin to cancel out drift
      Transformation T_W_S_new = submap_collection_ptr_->getActiveSubMapPose();
      T_W_D_ = T_W_S_new * T_W_S_old.inverse() * T_W_D_;
      T_world_robot = T_W_S_new * T_W_S_old.inverse() * T_world_robot;

      // Update the submap collection visualization in Rviz
      submap_vis_.publishSeparatedMesh(*submap_collection_ptr_, world_frame_,
                                       separated_mesh_pub_);

      // Play the rosbag (remove this once the system runs in real-time)
      rosbag_helper_.playRosbag();
    }

    // Create the new submap
    submap_collection_ptr_->createNewSubmap(T_world_robot, current_timestamp);
    if (debug_) {
      TfHelper::publishTransform(submap_collection_ptr_->getActiveSubMapPose(),
                                 world_frame_, "debug_T_world__new_submap",
                                 true, current_timestamp);
    }
  }

  // Lookup the sensor's pose in world frame
  Transformation T_world_sensor;
  if (use_gt_ptcloud_pose_from_sensor_tf_) {
    ROS_WARN_STREAM_THROTTLE(20, "Using ground truth pointcloud poses"
                                     << " provided by TF from " << world_frame_
                                     << " to "
                                     << pointcloud_msg->header.frame_id);
    transformer_.lookupTransform(pointcloud_msg->header.frame_id, world_frame_,
                                 current_timestamp, &T_world_sensor);
    if (debug_) {
      TfHelper::publishTransform(T_world_sensor, world_frame_,
                                 "debug_T_world_sensor", false,
                                 current_timestamp);
    }
  } else {
    // Get the pose of the sensor in world frame
    T_world_sensor = T_world_robot * T_robot_sensor_;
    if (debug_) {
      TfHelper::publishTransform(T_world_robot, world_frame_,
                                 "debug_T_world_robot", false,
                                 current_timestamp);
      TfHelper::publishTransform(T_robot_sensor_, world_frame_,
                                 "debug_T_robot_sensor", false,
                                 current_timestamp);
      TfHelper::publishTransform(T_world_sensor, world_frame_,
                                 "debug_T_world_sensor", false,
                                 current_timestamp);
    }
  }

  // Integrate the pointcloud
  pointcloud_processor_.integratePointcloud(pointcloud_msg, T_world_sensor);

  // Add the current pose to the submap's pose history
  submap_collection_ptr_->getActiveSubMapPtr()->addPoseToHistory(
      current_timestamp, T_world_robot);
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
}  // namespace voxgraph
