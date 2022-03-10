#ifndef VOXGRAPH_FRONTEND_VOXGRAPH_MAPPER_H_
#define VOXGRAPH_FRONTEND_VOXGRAPH_MAPPER_H_

#include <future>
#include <memory>
#include <string>

#include <panoptic_mapping_msgs/SubmapWithPlanes.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <voxblox_msgs/FilePath.h>
#include <voxblox_msgs/Submap.h>
#include <voxgraph_msgs/LoopClosure.h>

#include "voxgraph/common.h"
#include "voxgraph/frontend/frame_names.h"
#include "voxgraph/frontend/measurement_processors/gps_processor.h"
#include "voxgraph/frontend/measurement_processors/pointcloud_integrator.h"
#include "voxgraph/frontend/plane_collection/submap_stitcher.h"
#include "voxgraph/frontend/pose_graph_interface/pose_graph_manager.h"
#include "voxgraph/frontend/submap_collection/voxgraph_submap_collection.h"
#include "voxgraph/tools/data_servers/projected_map_server.h"
#include "voxgraph/tools/data_servers/submap_server.h"
#include "voxgraph/tools/rosbag_helper.h"
#include "voxgraph/tools/visualization/loop_closure_visuals.h"
#include "voxgraph/tools/visualization/submap_visuals.h"

namespace voxgraph {
class VoxgraphMapper {
 public:
  // Constructor & Destructor
  VoxgraphMapper(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  VoxgraphMapper(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
                 VoxgraphSubmap::Config submap_config,
                 voxblox::MeshIntegratorConfig mesh_config);
  ~VoxgraphMapper() = default;

  // ROS topic callbacks
  void loopClosureCallback(const voxgraph_msgs::LoopClosure& loop_closure_msg);
  virtual SubmapID submapCallback(const voxblox_msgs::Submap& submap_msg);
  virtual SubmapID submapWithPlanesCallback(
      const panoptic_mapping_msgs::SubmapWithPlanes& submap_planes_msg);
  // ROS service callbacks
  bool publishSeparatedMeshCallback(
      std_srvs::Empty::Request& request,     // NOLINT
      std_srvs::Empty::Response& response);  // NOLINT
  bool publishCombinedMeshCallback(
      std_srvs::Empty::Request& request,                            // NOLINT
      std_srvs::Empty::Response& response);                         // NOLINT
  bool optimizeGraphCallback(std_srvs::Empty::Request& request,     // NOLINT
                             std_srvs::Empty::Response& response);  // NOLINT
  bool optimizeSynchronouslySlidingPoseGraphCallback(
      std_srvs::Empty::Request& request,                        // NOLINT
      std_srvs::Empty::Response& response);                     // NOLINT
  bool finishMapCallback(std_srvs::Empty::Request& request,     // NOLINT
                         std_srvs::Empty::Response& response);  // NOLINT
  bool saveToFileCallback(
      voxblox_msgs::FilePath::Request& request,     // NOLINT
      voxblox_msgs::FilePath::Response& response);  // NOLINT
  bool savePoseHistoryToFileCallback(
      voxblox_msgs::FilePath::Request& request,     // NOLINT
      voxblox_msgs::FilePath::Response& response);  // NOLINT
  bool saveSeparatedMeshCallback(
      voxblox_msgs::FilePath::Request& request,     // NOLINT
      voxblox_msgs::FilePath::Response& response);  // NOLINT
  bool saveCombinedMeshCallback(
      voxblox_msgs::FilePath::Request& request,     // NOLINT
      voxblox_msgs::FilePath::Response& response);  // NOLINT
  bool saveOptimizationTimesCallback(
      voxblox_msgs::FilePath::Request& request,     // NOLINT
      voxblox_msgs::FilePath::Response& response);  // NOLINT
  bool pauseSlidingOptimizationCallback(
      std_srvs::SetBool::Request& request,     // NOLINT
      std_srvs::SetBool::Response& response);  // NOLINT
  bool pauseFullOptimizationCallback(
      std_srvs::SetBool::Request& request,     // NOLINT
      std_srvs::SetBool::Response& response);  // NOLINT

  const VoxgraphSubmapCollection& getSubmapCollection() {
    return *submap_collection_ptr_;
  }

  const PoseGraph::SolverSummaryList& getSlidingPoseGraphSolverSummaries() {
    return pose_graph_manager_.getSlidingPoseGraphSolverSummaries();
  }
  const PoseGraph::SolverSummaryList& getFullPoseGraphSolverSummaries() {
    return pose_graph_manager_.getFullPoseGraphSolverSummaries();
  }

 private:
  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Verbosity and debug mode
  bool verbose_;
  std::map<ros::Time, geometry_msgs::PoseStamped> poses_received_;
  // Flag and helper to automatically pause rosbags during graph optimization
  // NOTE: This is useful when playing bags faster than real-time or when
  //       experimenting with optimization settings
  bool auto_pause_rosbag_;
  RosbagHelper rosbag_helper_;

  // Interaction with ROS
  void subscribeToTopics();
  void advertiseTopics();
  void advertiseServices();
  void getParametersFromRos();

  // Map and pose publishing
  void publishMaps(const ros::Time& current_timestamp);
  void publishSubmapPoseTFs();
  double submap_pose_tf_publishing_period_s_;
  ros::Timer submap_pose_tf_publishing_timer_;

  // Asynchronous handle for the pose graph optimization thread
  std::future<void> optimization_async_handle_;

  // ROS topic subscribers
  std::string loop_closure_topic_;
  std::string submap_topic_;
  std::string submap_with_planes_topic_;
  int loop_closure_topic_queue_length_;
  int submap_topic_queue_length_;
  ros::Subscriber loop_closure_subscriber_;
  ros::Subscriber submap_subscriber_;
  ros::Subscriber submap_with_planes_subscriber_;
  // TODO(victorr): Add support for absolute pose measurements
  // ROS topic publishers
  ros::Publisher submap_mesh_pub_;
  ros::Publisher combined_mesh_pub_;
  ros::Publisher pose_history_pub_;
  ros::Publisher loop_closure_links_pub_;
  ros::Publisher loop_closure_axes_pub_;
  int publisher_queue_length_;

  // ROS service servers
  ros::ServiceServer publish_separated_mesh_srv_;
  ros::ServiceServer publish_combined_mesh_srv_;
  ros::ServiceServer optimize_graph_srv_;
  ros::ServiceServer optimize_sliding_graph_srv_;
  ros::ServiceServer finish_map_srv_;
  ros::ServiceServer save_to_file_srv_;
  ros::ServiceServer save_pose_history_to_file_srv_;
  ros::ServiceServer save_separated_mesh_srv_;
  ros::ServiceServer save_combined_mesh_srv_;
  ros::ServiceServer save_optimization_times_srv_;
  ros::ServiceServer pause_sliding_optimization_srv_;
  ros::ServiceServer pause_full_optimization_srv_;
  // TODO(victorr): Add srvs to receive absolute pose and loop closure updates

  // Constraints to be used
  bool odometry_constraints_enabled_;
  bool height_constraints_enabled_;
  bool plane_constraints_enabled_;
  bool pause_sliding_optimization_;
  bool pause_full_optimization_;

  // Instantiate the submap collection
  VoxgraphSubmap::Config submap_config_;
  VoxgraphSubmapCollection::Ptr submap_collection_ptr_;

  // Visualization tools
  SubmapVisuals submap_vis_;
  LoopClosureVisuals loop_closure_vis_;

  // Pose graph interaction
  double full_pose_graph_optimization_period_s_;
  ros::Timer full_pose_graph_optimization_timer_;
  int optimizeSlidingPoseGraph();
  int optimizeFullPoseGraph(bool skip_if_busy);
  PoseGraphManager pose_graph_manager_;
  SubmapStitcher submap_stitcher_;
  // Map servers, used to share the projected map and submaps with ROS nodes
  ProjectedMapServer projected_map_server_;
  SubmapServer submap_server_;

  // Class handling all frame names used to interface with ROS
  FrameNames frame_names_;
  std::string robocentric_robot_name_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_FRONTEND_VOXGRAPH_MAPPER_H_
