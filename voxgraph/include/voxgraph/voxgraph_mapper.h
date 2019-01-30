//
// Created by victor on 16.11.18.
//

#ifndef VOXGRAPH_VOXGRAPH_MAPPER_H
#define VOXGRAPH_VOXGRAPH_MAPPER_H

#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>

#include <cblox/core/common.h>
#include <cblox/core/submap_collection.h>
#include <cblox/mesh/submap_mesher.h>
#include <voxblox/core/common.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox_msgs/FilePath.h>
#include <voxblox_ros/mesh_vis.h>
#include <voxblox_ros/transformer.h>

#include "voxgraph/visualization/submap_visuals.h"
#include "voxgraph/voxgraph_submap.h"

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
  bool publishSeparatedMeshCallback(std_srvs::Empty::Request &request,
                                    std_srvs::Empty::Response &response);
  bool publishCombinedMeshCallback(std_srvs::Empty::Request &request,
                                   std_srvs::Empty::Response &response);
  bool saveToFileCallback(voxblox_msgs::FilePath::Request &request,
                          voxblox_msgs::FilePath::Response &response);

 private:
  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Verbosity
  bool verbose_ = true;

  // Interaction with ROS
  void subscribeToTopics();
  void advertiseTopics();
  void advertiseServices();
  void getParametersFromRos();

  // ROS topic subscribers
  unsigned int subscriber_queue_length_;
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

  // Use voxblox tsdf_integrator to integrate pointclouds into submaps
  // TODO(victorr): Use cblox SubmapCollection integrator method instead
  voxblox::TsdfIntegratorBase::Config tsdf_integrator_config_;
  std::unique_ptr<voxblox::FastTsdfIntegrator> tsdf_integrator_;

  // Use voxblox transformer to find transformations
  voxblox::Transformer transformer_;
  bool use_tf_to_get_pointcloud_poses_ = false;
  std::string odom_base_frame_ = "odom";

  // Visualization functions
  SubmapVisuals submap_vis_;

  // TODO(victorr): Integrate these variables into the structure nicely
  ros::Time current_submap_creation_stamp_ = {};
  ros::Duration submap_creation_interval_ = ros::Duration(20);  // In seconds
};
}  // namespace voxgraph

#endif  // VOXGRAPH_VOXGRAPH_MAPPER_H
