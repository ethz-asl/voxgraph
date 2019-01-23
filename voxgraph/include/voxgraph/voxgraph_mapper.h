//
// Created by victor on 16.11.18.
//

#ifndef VOXGRAPH_VOXGRAPH_MAPPER_H
#define VOXGRAPH_VOXGRAPH_MAPPER_H

#include <geometry_msgs/PoseWithCovarianceStamped.h>
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

namespace voxgraph {
class VoxgraphMapper {
 public:
  // Constructor & Destructor
  VoxgraphMapper(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
  ~VoxgraphMapper() = default;

  // ROS topic callbacks
  void odometryCallback(const nav_msgs::Odometry::ConstPtr &odometry_msg);
  void absolutePoseCallback(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg);
  void pointcloudCallback(
      const sensor_msgs::PointCloud2::ConstPtr &pointcloud_msg);

  // ROS service callbacks
  bool publishSeparatedMeshCallback(std_srvs::Empty::Request &request,
                                    std_srvs::Empty::Response &response);
  bool publishCombinedMeshCallback(std_srvs::Empty::Request &request,
                                   std_srvs::Empty::Response &response);
  bool saveToFileCallback(voxblox_msgs::FilePath::Request &request,
                          voxblox_msgs::FilePath::Response &response);
  bool optimizeGraphCallback(std_srvs::Empty::Request &request,
                             std_srvs::Empty::Response &response);

  // TODO(victorr): Remove this once the visualization tools
  //                get their own class
  static void publishMesh(const voxblox::MeshLayer::Ptr &mesh_layer_ptr,
                          const ros::Publisher &publisher,
                          const voxblox::ColorMode &color_mode,
                          const std::string &reference_frame = "world");

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
  int subscriber_queue_length_;
  ros::Subscriber odometry_subscriber_;
  ros::Subscriber absolute_pose_subscriber_;
  ros::Subscriber pointcloud_subscriber_;

  // ROS topic publishers
  ros::Publisher separated_mesh_pub_;
  ros::Publisher combined_mesh_pub_;

  // ROS service servers
  ros::ServiceServer publish_separated_mesh_srv_;
  ros::ServiceServer publish_combined_mesh_srv_;
  ros::ServiceServer save_to_file_srv_;
  ros::ServiceServer optimize_graph_srv_;

  // Instantiate a TSDF submap collection
  cblox::TsdfMap::Config tsdf_map_config_;
  cblox::SubmapCollection<cblox::TsdfSubmap> tsdf_submap_collection_;

  // Use voxblox tsdf_integrator to integrate pointclouds into submaps
  voxblox::TsdfIntegratorBase::Config tsdf_integrator_config_;
  std::unique_ptr<voxblox::FastTsdfIntegrator> tsdf_integrator_;

  // Instantiate a cblox mesher to extract meshes
  // from the TSDF submap collection
  cblox::MeshIntegratorConfig mesh_integrator_config_;
  cblox::SubmapMesher tsdf_submap_mesher_;

  // Use voxblox transformer to find transformations
  voxblox::Transformer transformer_;
  bool use_tf_to_get_pointcloud_poses_ = false;

  // Imperfect odometry simulator
  geometry_msgs::PoseStamped odom_simulator_pose_stamped_;
  double odom_simulator_noise_linear_vel_mean_ = 0;
  double odom_simulator_noise_linear_vel_stddev_ = 0;
  double odom_simulator_noise_angular_vel_mean_ = 0;
  double odom_simulator_noise_angular_vel_stddev_ = 0;
  std::string odom_base_frame_ = "odom";
  std::default_random_engine odom_simulator_noise_generator_;
  std::normal_distribution<double> odom_simulator_noise_linear_vel_dist_;
  std::normal_distribution<double> odom_simulator_noise_angular_vel_dist_;

  // Visualization functions
  enum MeshType { SEPARATED_MESH, COMBINED_MESH };
  void generateMesh(const MeshType &mesh_type,
                    const voxblox::MeshLayer::Ptr &mesh_layer_ptr);
  // Convenience function to easily publish
  // debug visuals for the entire collection
  void publishMesh(MeshType);

  // TODO(victorr): Integrate these variables into the structure nicely
  ros::Time current_submap_creation_stamp_ = {};
  ros::Duration submap_creation_interval_ = ros::Duration(20);  // In seconds
};
}  // namespace voxgraph

#endif  // VOXGRAPH_VOXGRAPH_MAPPER_H
