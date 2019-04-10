//
// Created by victor on 16.11.18.
//

#ifndef VOXGRAPH_FRONTEND_VOXGRAPH_MAPPER_H_
#define VOXGRAPH_FRONTEND_VOXGRAPH_MAPPER_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <voxblox_msgs/FilePath.h>
#include <voxblox_ros/transformer.h>
#include <memory>
#include <string>
#include "voxgraph/common.h"
#include "voxgraph/frontend/measurement_processors/gps_processor.h"
#include "voxgraph/frontend/measurement_processors/pointcloud_processor.h"
#include "voxgraph/frontend/pose_graph_interface.h"
#include "voxgraph/frontend/submap_collection/voxgraph_submap_collection.h"
#include "voxgraph/tools/rosbag_helper.h"
#include "voxgraph/tools/visualization/submap_visuals.h"

namespace voxgraph {
class VoxgraphMapper {
 public:
  // Constructor & Destructor
  VoxgraphMapper(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
  ~VoxgraphMapper() = default;

  // ROS topic callbacks
  void pointcloudCallback(
      const sensor_msgs::PointCloud2::ConstPtr &pointcloud_msg);

  // ROS service callbacks
  bool publishSeparatedMeshCallback(
      std_srvs::Empty::Request &request,     // NOLINT
      std_srvs::Empty::Response &response);  // NOLINT
  bool publishCombinedMeshCallback(
      std_srvs::Empty::Request &request,     // NOLINT
      std_srvs::Empty::Response &response);  // NOLINT
  bool saveToFileCallback(
      voxblox_msgs::FilePath::Request &request,     // NOLINT
      voxblox_msgs::FilePath::Response &response);  // NOLINT

  const VoxgraphSubmapCollection &getSubmapCollection() {
    return *submap_collection_ptr_;
  }

 private:
  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Verbosity and debug mode
  bool verbose_;
  bool debug_;

  // Helper to pause/unpause rosbags during initial, non real-time testing
  RosbagHelper rosbag_helper_;

  // Interaction with ROS
  void subscribeToTopics();
  void advertiseTopics();
  void advertiseServices();
  void getParametersFromRos();

  // ROS topic subscribers
  int subscriber_queue_length_;
  std::string pointcloud_topic_;
  ros::Subscriber pointcloud_subscriber_;

  // ROS topic publishers
  ros::Publisher separated_mesh_pub_;
  ros::Publisher combined_mesh_pub_;

  // ROS service servers
  ros::ServiceServer publish_separated_mesh_srv_;
  ros::ServiceServer publish_combined_mesh_srv_;
  ros::ServiceServer save_to_file_srv_;
  // TODO(victorr): Add srvs to receive absolute pose and loop closure updates

  // Visualization tools
  SubmapVisuals submap_vis_;

  // Instantiate the submap collection
  VoxgraphSubmap::Config submap_config_;
  VoxgraphSubmapCollection::Ptr submap_collection_ptr_;

  // Interface to ease interaction with the pose graph
  PoseGraphInterface pose_graph_interface_;

  // Measurement processors
  PointcloudProcessor pointcloud_processor_;

  // Voxblox transformer used to lookup transforms from the TF tree or rosparams
  voxblox::Transformer transformer_;

  // Coordinate frame naming convention:
  // W: World frame
  // R: The body frame of the robot
  // D: A fictional frame that is used cancel out the odometry drift
  std::string world_frame_;
  bool lookup_T_world_robot(ros::Time timestamp,
                            Transformation *T_world_robot_ptr);

  // Static transform from body to sensor frame
  Transformation T_robot_sensor_;

  // Transform from the world frame to the fictional drifting odometry origin
  // such that T_W_D * T_D_R = the drift free robot pose in world frame,
  // where T_D_R corresponds to the received odometry TF
  // NOTE: T_W_D is initialized to the robot's starting pose and gets updated
  //       when the active submap jumps following a pose graph optimization step
  Transformation T_W_D_;
  std::string odom_tf_frame_;

  // Whether to use ground truth T_world__sensor,
  // instead of its estimated pose (for validation purposes)
  bool use_gt_ptcloud_pose_from_sensor_tf_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_FRONTEND_VOXGRAPH_MAPPER_H_
