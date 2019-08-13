#include "voxgraph/frontend/voxgraph_mapper.h"
#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_xml.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/MarkerArray.h>
#include <memory>
#include <string>
#include <thread>
#include <limits>
#include "voxgraph/frontend/submap_collection/submap_timeline.h"
#include "voxgraph/tools/submap_registration_helper.h"
#include "voxgraph/tools/tf_helper.h"

namespace voxgraph {
VoxgraphMapper::VoxgraphMapper(const ros::NodeHandle &nh,
                               const ros::NodeHandle &nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      verbose_(false),
      auto_pause_rosbag_(false),
      rosbag_helper_(nh),
      subscriber_queue_length_(100),
      pointcloud_topic_("pointcloud"),
      registration_constraints_enabled_(false),
      odometry_constraints_enabled_(false),
      height_constraints_enabled_(false),
      submap_collection_ptr_(
          std::make_shared<VoxgraphSubmapCollection>(submap_config_)),
      pose_graph_interface_(nh_private, submap_collection_ptr_),
      pointcloud_processor_(submap_collection_ptr_),
      projected_map_server_(nh_private),
      submap_server_(nh_private),
      loop_closure_edge_server_(nh_private),
      submap_vis_(submap_config_),
      use_icp_refinement_(true),
      map_tracker_(submap_collection_ptr_,
                   FrameNames::fromRosParams(nh_private), verbose_) {
  // Setup interaction with ROS
  getParametersFromRos();
  subscribeToTopics();
  advertiseTopics();
  advertiseServices();
}

void VoxgraphMapper::getParametersFromRos() {
  nh_private_.param("verbose", verbose_, verbose_);
  pose_graph_interface_.setVerbosity(verbose_);
  map_tracker_.setVerbosity(verbose_);

  nh_private_.param("subscriber_queue_length", subscriber_queue_length_,
                    subscriber_queue_length_);
  nh_private_.param("pointcloud_topic", pointcloud_topic_, pointcloud_topic_);

  // Get the submap creation interval as a ros::Duration
  double interval_temp;
  if (nh_private_.getParam("submap_creation_interval", interval_temp)) {
    submap_collection_ptr_->setSubmapCreationInterval(
        ros::Duration(interval_temp));
  }

  // Read whether or not to auto pause the rosbag during graph optimization
  nh_private_.param("auto_pause_rosbag", auto_pause_rosbag_,
                    auto_pause_rosbag_);

  //  // Load the transform from the base frame to the sensor frame, if
  //  available
  //  // NOTE: If the transform is not specified through ROS params, voxgraph
  //  will
  //  //       attempt to get if from TFs
  //  XmlRpc::XmlRpcValue T_base_sensor_xml;
  //  get_sensor_calibration_from_tfs_ =
  //      !nh_private_.getParam("T_base_sensor", T_base_sensor_xml);
  //  if (!get_sensor_calibration_from_tfs_) {
  //    kindr::minimal::xmlRpcToKindr(T_base_sensor_xml, &T_B_C_);
  //  }
  //  ROS_INFO_STREAM(
  //      "Using transform from pointcloud sensor to robot base "
  //      "link from "
  //      << (get_sensor_calibration_from_tfs_ ? "TFs" : "ROS params"));

  // Read the measurement params from their sub-namespace
  ros::NodeHandle nh_measurement_params(nh_private_, "measurements");
  nh_measurement_params.param("submap_registration/enabled",
                              registration_constraints_enabled_,
                              registration_constraints_enabled_);
  ROS_INFO_STREAM_COND(
      verbose_,
      "Submap registration constraints: "
          << (registration_constraints_enabled_ ? "enabled" : "disabled"));
  nh_measurement_params.param("odometry/enabled", odometry_constraints_enabled_,
                              odometry_constraints_enabled_);
  ROS_INFO_STREAM_COND(
      verbose_,
      "Odometry constraints: " << (odometry_constraints_enabled_ ? "enabled"
                                                                 : "disabled"));
  nh_measurement_params.param("height/enabled", height_constraints_enabled_,
                              height_constraints_enabled_);
  ROS_INFO_STREAM_COND(
      verbose_, "Height constraints: "
                    << (height_constraints_enabled_ ? "enabled" : "disabled"));
  pose_graph_interface_.setMeasurementConfigFromRosParams(
      nh_measurement_params);

  // Read TSDF integrator params from their sub-namespace
  ros::NodeHandle nh_tsdf_params(nh_private_, "tsdf_integrator");
  pointcloud_processor_.setTsdfIntegratorConfigFromRosParam(nh_tsdf_params);
}

void VoxgraphMapper::subscribeToTopics() {
  pointcloud_subscriber_ =
      nh_.subscribe(pointcloud_topic_, subscriber_queue_length_,
                    &VoxgraphMapper::pointcloudCallback, this);
  map_tracker_.subscribeToTopics(
      nh_, nh_private_.param<std::string>("odometry_input_topic", ""),
      nh_private_.param<std::string>("imu_biases_topic", ""));
}

void VoxgraphMapper::advertiseTopics() {
  separated_mesh_pub_ = nh_private_.advertise<visualization_msgs::Marker>(
      "separated_mesh", subscriber_queue_length_, true);
  combined_mesh_pub_ = nh_private_.advertise<visualization_msgs::Marker>(
      "combined_mesh", subscriber_queue_length_, true);
  pose_history_pub_ =
      nh_private_.advertise<nav_msgs::Path>("pose_history", 1, true);
  map_tracker_.advertiseTopics(
      nh_private_,
      nh_private_.param<std::string>("odometry_output_topic", "odometry"));
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
  submap_vis_.publishSeparatedMesh(*submap_collection_ptr_,
                                   map_tracker_.getFrameNames().mission_frame,
                                   separated_mesh_pub_);
  return true;  // Tell ROS it succeeded
}

bool VoxgraphMapper::publishCombinedMeshCallback(
    std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {
  submap_vis_.publishCombinedMesh(*submap_collection_ptr_,
                                  map_tracker_.getFrameNames().mission_frame,
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
    const sensor_msgs::PointCloud2::Ptr &pointcloud_msg) {
  // Lookup the robot pose at the time of the pointcloud message
  ros::Time current_timestamp = pointcloud_msg->header.stamp;
  if (!map_tracker_.updateToTime(current_timestamp,
                                 pointcloud_msg->header.frame_id)) {
    // If the pose cannot be found, the pointcloud is skipped
    ROS_WARN_STREAM("Skipping pointcloud since the poses at time "
                    << current_timestamp << " could not be lookup up.");
    return;
  }

  // Check if it's time to create a new submap
  if (submap_collection_ptr_->shouldCreateNewSubmap(current_timestamp)) {
    // Add the finished submap to the pose graph, optimize and correct for drift
    // NOTE: We only add submaps to the pose graph once they're finished to
    //       avoid generating their ESDF more than once
    if (!submap_collection_ptr_->empty()) {
      // Automatically pause the rosbag if requested
      if (auto_pause_rosbag_) {
        rosbag_helper_.pauseRosbag();
      }

      // Add the finished submap to the pose graph, including an odometry link
      SubmapID finished_submap_id = submap_collection_ptr_->getActiveSubmapID();
      pose_graph_interface_.addSubmap(finished_submap_id,
                                      odometry_constraints_enabled_);

      // Constrain the height
      // NOTE: No constraint is added for the 1st since its pose is fixed
      if (height_constraints_enabled_ && submap_collection_ptr_->size() > 1) {
        // This assumes that the height estimate from the odometry source is
        // really good (e.g. when it fuses an altimeter)
        double current_height = map_tracker_.get_T_O_B().getPosition().z();
        pose_graph_interface_.addHeightMeasurement(finished_submap_id,
                                                   current_height);
      }

      // Optimize the pose graph
      ROS_INFO("Optimizing the pose graph");
      if (registration_constraints_enabled_) {
        pose_graph_interface_.updateRegistrationConstraints();
      }
      pose_graph_interface_.optimize();

      // Remember the pose of the current submap (used later to eliminate drift)
      Transformation T_M_S_before =
          submap_collection_ptr_->getActiveSubmapPose();

      // Update the submap poses
      pose_graph_interface_.updateSubmapCollectionPoses();

      // Update the fictional odometry origin to cancel out drift
      Transformation T_M_S_after =
          submap_collection_ptr_->getActiveSubmapPose();
      map_tracker_.updateWithLoopClosure(T_M_S_before, T_M_S_after);

      // Only auto publish the updated meshes if there are subscribers
      // NOTE: Users can request new meshes at any time through service calls
      //       so there's no point in publishing them just in case
      if (combined_mesh_pub_.getNumSubscribers() > 0) {
        std::thread combined_mesh_thread(
            &SubmapVisuals::publishCombinedMesh, &submap_vis_,
            *submap_collection_ptr_, map_tracker_.getFrameNames().mission_frame,
            combined_mesh_pub_);
        combined_mesh_thread.detach();
      }
      if (separated_mesh_pub_.getNumSubscribers() > 0) {
        std::thread separated_mesh_thread(
            &SubmapVisuals::publishSeparatedMesh, &submap_vis_,
            *submap_collection_ptr_, map_tracker_.getFrameNames().mission_frame,
            separated_mesh_pub_);
        separated_mesh_thread.detach();
      }
      submap_server_.publishSubmap(
          submap_collection_ptr_->getSubmap(finished_submap_id),
          current_timestamp);
      projected_map_server_.publishProjectedMap(*submap_collection_ptr_,
                                                current_timestamp);
      loop_closure_edge_server_.publishLoopClosureEdges(
          pose_graph_interface_, *submap_collection_ptr_, current_timestamp);

      // Resume playing  the rosbag
      if (auto_pause_rosbag_) {
        rosbag_helper_.playRosbag();
      }
    }

    // Create the new submap
    submap_collection_ptr_->createNewSubmap(map_tracker_.get_T_M_B(),
                                            current_timestamp);
    TfHelper::publishTransform(submap_collection_ptr_->getActiveSubmapPose(),
                               map_tracker_.getFrameNames().mission_frame,
                               "new_submap_origin", true, current_timestamp);
  }

  // Refine the camera pose using scan to submap matching
  if (use_icp_refinement_) {
    map_tracker_.registerPointcloud(pointcloud_msg);
  }

  // Integrate the pointcloud
  pointcloud_processor_.integratePointcloud(pointcloud_msg,
                                            map_tracker_.get_T_M_C());

  // Add the current pose to the submap's pose history
  submap_collection_ptr_->getActiveSubmapPtr()->addPoseToHistory(
      current_timestamp, map_tracker_.get_T_M_B());

  // Publish the odometry
  map_tracker_.publishOdometry();

  // Publish the frames
  map_tracker_.publishTFs();

  // Publish the pose history
  if (pose_history_pub_.getNumSubscribers() > 0) {
    submap_vis_.publishPoseHistory(*submap_collection_ptr_,
                                   map_tracker_.getFrameNames().mission_frame,
                                   pose_history_pub_);
  }
}
}  // namespace voxgraph
