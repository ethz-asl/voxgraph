#include "voxgraph/frontend/voxgraph_mapper.h"
#include <minkindr_conversions/kindr_xml.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <memory>
#include <string>
#include <thread>
#include "voxgraph/frontend/submap_collection/submap_timeline.h"
#include "voxgraph/tools/submap_registration_helper.h"
#include "voxgraph/tools/tf_helper.h"

namespace voxgraph {
VoxgraphMapper::VoxgraphMapper(const ros::NodeHandle &nh,
                               const ros::NodeHandle &nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      verbose_(false),
      debug_(false),
      auto_pause_rosbag_(false),
      subscriber_queue_length_(100),
      pointcloud_topic_("pointcloud"),
      transformer_(nh, nh_private),
      mission_frame_("mission"),
      odom_frame_("odom"),
      base_frame_("base_link"),
      use_tf_transforms_(false),
      submap_collection_ptr_(
          std::make_shared<VoxgraphSubmapCollection>(submap_config_)),
      pose_graph_interface_(nh_private, submap_collection_ptr_),
      pointcloud_processor_(submap_collection_ptr_),
      submap_vis_(submap_config_),
      registration_constraints_enabled_(false),
      odometry_constraints_enabled_(false),
      height_constraints_enabled_(false),
      rosbag_helper_(nh),
      scan_to_map_registerer_(submap_collection_ptr_) {
  // Setup interaction with ROS
  getParametersFromRos();
  subscribeToTopics();
  advertiseTopics();
  advertiseServices();

  // Publish the initial transform from the mission to the drifting odom frame
  TfHelper::publishTransform(voxblox::Transformation(), mission_frame_,
                             odom_frame_corrected_, true);
}

void VoxgraphMapper::getParametersFromRos() {
  nh_private_.param("debug", debug_, debug_);
  nh_private_.param("verbose", verbose_, verbose_);
  pose_graph_interface_.setVerbosity(verbose_);
  //  scan_to_map_registerer_.setVerbosity(verbose_);
  // TODO(victorr): Revert this
  scan_to_map_registerer_.setVerbosity(true);

  nh_private_.param("subscriber_queue_length", subscriber_queue_length_,
                    subscriber_queue_length_);
  nh_private_.param("pointcloud_topic", pointcloud_topic_, pointcloud_topic_);
  nh_private_.param("mission_frame", mission_frame_, mission_frame_);
  nh_private_.param("odom_frame", odom_frame_, odom_frame_);
  nh_private_.param("base_frame", base_frame_, base_frame_);
  nh_private_.param("odom_frame_corrected", odom_frame_corrected_,
                    odom_frame_corrected_);
  nh_private_.param("base_frame_corrected", base_frame_corrected_,
                    base_frame_corrected_);
  nh_private_.param("sensor_frame_corrected", sensor_frame_corrected_,
                    sensor_frame_corrected_);
  nh_private_.param("use_tf_transforms", use_tf_transforms_,
                    use_tf_transforms_);

  // Get the submap creation interval as a ros::Duration
  double interval_temp;
  if (nh_private_.getParam("submap_creation_interval", interval_temp)) {
    submap_collection_ptr_->setSubmapCreationInterval(
        ros::Duration(interval_temp));
  }

  // Read whether or not to auto pause the rosbag during graph optimization
  nh_private_.param("auto_pause_rosbag", auto_pause_rosbag_,
                    auto_pause_rosbag_);

  // Load the transform from the base frame to the sensor frame
  XmlRpc::XmlRpcValue T_base_sensor_xml;
  CHECK(nh_private_.getParam("T_base_sensor", T_base_sensor_xml))
      << "The transform from the base frame to the sensor frame "
      << "T_base_sensor is required but was not set.";
  kindr::minimal::xmlRpcToKindr(T_base_sensor_xml, &T_B_C_);

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
}

void VoxgraphMapper::advertiseTopics() {
  separated_mesh_pub_ = nh_private_.advertise<visualization_msgs::Marker>(
      "separated_mesh", subscriber_queue_length_, true);
  combined_mesh_pub_ = nh_private_.advertise<visualization_msgs::Marker>(
      "combined_mesh", subscriber_queue_length_, true);
  pose_history_pub_ =
      nh_private_.advertise<nav_msgs::Path>("pose_history", 1, true);
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
  submap_vis_.publishSeparatedMesh(*submap_collection_ptr_, mission_frame_,
                                   separated_mesh_pub_);
  return true;  // Tell ROS it succeeded
}

bool VoxgraphMapper::publishCombinedMeshCallback(
    std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {
  submap_vis_.publishCombinedMesh(*submap_collection_ptr_, mission_frame_,
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
  Transformation T_odom_base;
  if (!lookup_T_odom_base(current_timestamp, &T_odom_base)) {
    // If the pose cannot be found, the pointcloud is skipped
    ROS_WARN_STREAM("Skipping pointcloud since the robot pose at time "
                    << current_timestamp << " is unknown.");
    return;
  }
  Transformation T_mission_base = T_M_L_ * T_odom_base;

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
        double current_height = T_odom_base.getPosition().z();
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
      Transformation T_W_S_old = submap_collection_ptr_->getActiveSubmapPose();

      // Update the submap poses
      pose_graph_interface_.updateSubmapCollectionPoses();

      // Update the fictional odometry origin to cancel out drift
      Transformation T_W_S_new = submap_collection_ptr_->getActiveSubmapPose();
      T_M_L_ = T_W_S_new * T_W_S_old.inverse() * T_M_L_,
      T_mission_base = T_W_S_new * T_W_S_old.inverse() * T_mission_base;
      ROS_INFO_STREAM("Applying pose correction:\n"
                      << T_W_S_new * T_W_S_old.inverse());

      // Publish the new corrected odom frame
      TfHelper::publishTransform(T_M_L_, mission_frame_, odom_frame_corrected_,
                                 true, current_timestamp);

      // Only auto publish the updated meshes if there are subscribers
      // NOTE: Users can request new meshes at any time through service calls
      //       so there's no point in publishing them just in case
      if (combined_mesh_pub_.getNumSubscribers() > 0) {
        std::thread combined_mesh_thread(&SubmapVisuals::publishCombinedMesh,
                                         &submap_vis_, *submap_collection_ptr_,
                                         mission_frame_, combined_mesh_pub_);
        combined_mesh_thread.detach();
      }
      if (separated_mesh_pub_.getNumSubscribers() > 0) {
        std::thread separated_mesh_thread(&SubmapVisuals::publishSeparatedMesh,
                                          &submap_vis_, *submap_collection_ptr_,
                                          mission_frame_, separated_mesh_pub_);
        separated_mesh_thread.detach();
      }

      // Resume playing  the rosbag
      if (auto_pause_rosbag_) {
        rosbag_helper_.playRosbag();
      }
    }

    // Create the new submap
    submap_collection_ptr_->createNewSubmap(T_mission_base, current_timestamp);
    if (debug_) {
      TfHelper::publishTransform(submap_collection_ptr_->getActiveSubmapPose(),
                                 mission_frame_, "new_submap_origin", true,
                                 current_timestamp);
    }
  }

  // Lookup the sensor's pose through TFs if appropriate
  if (use_tf_transforms_) {
    transformer_.lookupTransform(pointcloud_msg->header.frame_id, base_frame_,
                                 current_timestamp, &T_B_C_);
  }

  // Get the pose of the sensor in mission frame
  Transformation T_mission_sensor = T_mission_base * T_B_C_;
  if (debug_) {
    TfHelper::publishTransform(T_mission_base, mission_frame_,
                               base_frame_corrected_, false, current_timestamp);
    TfHelper::publishTransform(T_B_C_, base_frame_corrected_,
                               sensor_frame_corrected_, true,
                               current_timestamp);
  }

  // Refine the camera pose using scan to submap matching
  Transformation T_mission_sensor_refined;
  bool pose_refinement_successful = scan_to_map_registerer_.refineSensorPose(
      pointcloud_msg, T_mission_sensor, &T_mission_sensor_refined);
  if (pose_refinement_successful) {
    // Update the robot and sensor pose
    T_mission_sensor = T_mission_sensor_refined;
    T_mission_base = T_mission_sensor * T_B_C_.inverse();
    // Update and publish the corrected odometry frame
    T_M_L_ = T_mission_base * T_odom_base.inverse();
    TfHelper::publishTransform(T_M_L_, mission_frame_, odom_frame_corrected_,
                               true, current_timestamp);
  } else {
    ROS_WARN("Pose refinement failed");
  }

  // Integrate the pointcloud
  pointcloud_processor_.integratePointcloud(pointcloud_msg, T_mission_sensor);

  // Add the current pose to the submap's pose history
  submap_collection_ptr_->getActiveSubmapPtr()->addPoseToHistory(
      current_timestamp, T_mission_base);

  // Publish the pose history
  if (pose_history_pub_.getNumSubscribers() > 0) {
    submap_vis_.publishPoseHistory(*submap_collection_ptr_, mission_frame_,
                                   pose_history_pub_);
  }
}

bool VoxgraphMapper::lookup_T_odom_base(ros::Time timestamp,
                                        Transformation *T_odom_base) {
  CHECK_NOTNULL(T_odom_base);
  Transformation T_odom_base_received;
  double t_waited = 0;   // Total time spent waiting for the updated pose
  double t_max = 0.20;   // Maximum time to wait before giving up
  const ros::Duration timeout(0.005);  // Timeout between each update attempt
  while (t_waited < t_max) {
    if (transformer_.lookupTransform(base_frame_, odom_frame_, timestamp,
                                     &T_odom_base_received)) {
      *T_odom_base = T_odom_base_received;
      return true;
    }
    timeout.sleep();
    t_waited += timeout.toSec();
  }
  ROS_WARN("Waited %.3fs, but still could not get the TF from %s to %s",
           t_waited, base_frame_.c_str(), odom_frame_.c_str());
  return false;
}
}  // namespace voxgraph
