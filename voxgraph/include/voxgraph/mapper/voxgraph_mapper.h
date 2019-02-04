//
// Created by victor on 16.11.18.
//

#ifndef VOXGRAPH_MAPPER_VOXGRAPH_MAPPER_H_
#define VOXGRAPH_MAPPER_VOXGRAPH_MAPPER_H_

#include <cblox/core/common.h>
#include <cblox/core/submap_collection.h>
#include <cblox/mesh/submap_mesher.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <voxblox/core/common.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox_msgs/FilePath.h>
#include <voxblox_ros/mesh_vis.h>
#include <voxblox_ros/transformer.h>
#include <memory>
#include <string>
#include "voxgraph/mapper/voxgraph_submap.h"
#include "voxgraph/visualization/submap_visuals.h"

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

 private:
  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Verbosity and debug mode
  bool verbose_;
  bool debug_;

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

  // Instantiate a TSDF submap collection
  VoxgraphSubmap::Config submap_config_;
  cblox::SubmapCollection<VoxgraphSubmap> submap_collection_;

  // Control new submap creation
  bool shouldCreateNewSubmap(const ros::Time &current_time);
  ros::Time current_submap_creation_stamp_;
  ros::Duration submap_creation_interval_;

  // Tools to integrate the pointclouds into submaps
  voxblox::TsdfIntegratorBase::Config tsdf_integrator_config_;
  std::unique_ptr<voxblox::FastTsdfIntegrator> tsdf_integrator_;
  void integratePointcloud(
      const sensor_msgs::PointCloud2::ConstPtr &pointcloud_msg,
      const voxblox::Transformation &T_world__sensor);

  // Transformations and related tools
  voxblox::Transformer transformer_;
  std::string odom_tf_frame_;
  std::string world_frame_;
  voxblox::Transformation T_odom__sensor_;
  voxblox::Transformation T_world__odom_;
  void updateToOdomAt(ros::Time timestamp);

  // Whether to directly use ground truth T_world__sensor, or
  // T_world__sensor = T_world__noisy_odom * T_odom__sensor
  bool use_gt_ptcloud_pose_from_sensor_tf_;

  // Visualization functions
  SubmapVisuals submap_vis_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_MAPPER_VOXGRAPH_MAPPER_H_
