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
      imu_biases_topic_("imu_biases"),
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
      transformer_(nh, nh_private),
      frame_names_(FrameNames::fromRosParams(nh_private)),
      use_tf_transforms_(false),
      scan_to_map_registerer_(submap_collection_ptr_) {
  // Setup interaction with ROS
  getParametersFromRos();
  subscribeToTopics();
  advertiseTopics();
  advertiseServices();
}

void VoxgraphMapper::getParametersFromRos() {
  nh_private_.param("verbose", verbose_, verbose_);
  pose_graph_interface_.setVerbosity(verbose_);
  //  scan_to_map_registerer_.setVerbosity(verbose_);
  // TODO(victorr): Revert this
  scan_to_map_registerer_.setVerbosity(true);

  nh_private_.param("subscriber_queue_length", subscriber_queue_length_,
                    subscriber_queue_length_);
  nh_private_.param("pointcloud_topic", pointcloud_topic_, pointcloud_topic_);
  nh_private_.param("imu_biases_topic", imu_biases_topic_, imu_biases_topic_);
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
  imu_biases_subscriber_ = nh_.subscribe(
      imu_biases_topic_, 1, &VoxgraphMapper::imuBiasesCallback, this);
}

void VoxgraphMapper::advertiseTopics() {
  separated_mesh_pub_ = nh_private_.advertise<visualization_msgs::Marker>(
      "separated_mesh", subscriber_queue_length_, true);
  combined_mesh_pub_ = nh_private_.advertise<visualization_msgs::Marker>(
      "combined_mesh", subscriber_queue_length_, true);
  pose_history_pub_ =
      nh_private_.advertise<nav_msgs::Path>("pose_history", 1, true);
  odom_with_imu_biases_pub_ =
      nh_private_.advertise<maplab_msgs::OdometryWithImuBiases>("odom", 1,
                                                                false);
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
  submap_vis_.publishSeparatedMesh(
      *submap_collection_ptr_, frame_names_.mission_frame, separated_mesh_pub_);
  return true;  // Tell ROS it succeeded
}

bool VoxgraphMapper::publishCombinedMeshCallback(
    std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {
  submap_vis_.publishCombinedMesh(
      *submap_collection_ptr_, frame_names_.mission_frame, combined_mesh_pub_);
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
  Transformation T_mission_base = T_M_L_ * T_L_O_ * T_odom_base;

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
      Transformation T_M_S_old = submap_collection_ptr_->getActiveSubmapPose();

      // Update the submap poses
      pose_graph_interface_.updateSubmapCollectionPoses();

      // Update the fictional odometry origin to cancel out drift
      Transformation T_M_S_new = submap_collection_ptr_->getActiveSubmapPose();
      T_M_L_ = T_M_S_new * T_M_S_old.inverse() * T_M_L_,
      T_mission_base = T_M_S_new * T_M_S_old.inverse() * T_mission_base;
      ROS_INFO_STREAM("Applying pose correction:\n"
                      << T_M_S_new * T_M_S_old.inverse());

      // Only auto publish the updated meshes if there are subscribers
      // NOTE: Users can request new meshes at any time through service calls
      //       so there's no point in publishing them just in case
      if (combined_mesh_pub_.getNumSubscribers() > 0) {
        std::thread combined_mesh_thread(&SubmapVisuals::publishCombinedMesh,
                                         &submap_vis_, *submap_collection_ptr_,
                                         frame_names_.mission_frame,
                                         combined_mesh_pub_);
        combined_mesh_thread.detach();
      }
      if (separated_mesh_pub_.getNumSubscribers() > 0) {
        std::thread separated_mesh_thread(&SubmapVisuals::publishSeparatedMesh,
                                          &submap_vis_, *submap_collection_ptr_,
                                          frame_names_.mission_frame,
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
    submap_collection_ptr_->createNewSubmap(T_mission_base, current_timestamp);
    TfHelper::publishTransform(submap_collection_ptr_->getActiveSubmapPose(),
                               frame_names_.mission_frame, "new_submap_origin",
                               true, current_timestamp);
  }

  // Lookup the sensor's pose through TFs if appropriate
  if (use_tf_transforms_) {
    transformer_.lookupTransform(pointcloud_msg->header.frame_id,
                                 frame_names_.base_link_frame,
                                 current_timestamp, &T_B_C_);
  }

  // Get the pose of the sensor in mission frame
  Transformation T_mission_sensor = T_mission_base * T_B_C_;

  // Refine the camera pose using scan to submap matching
  Transformation T_mission_sensor_refined;
  bool pose_refinement_successful = scan_to_map_registerer_.refineSensorPose(
      pointcloud_msg, T_mission_sensor, &T_mission_sensor_refined);
  if (pose_refinement_successful) {
    // Update the robot and sensor pose
    T_mission_sensor = T_mission_sensor_refined;
    T_mission_base = T_mission_sensor * T_B_C_.inverse();
    // Update and publish the corrected odometry frame
    T_L_O_ = T_M_L_.inverse() * T_mission_base * T_odom_base.inverse();
  } else {
    ROS_WARN("Pose refinement failed");
  }

  // Integrate the pointcloud
  pointcloud_processor_.integratePointcloud(pointcloud_msg, T_mission_sensor);

  // Add the current pose to the submap's pose history
  submap_collection_ptr_->getActiveSubmapPtr()->addPoseToHistory(
      current_timestamp, T_mission_base);

  // Publish the odometry
  constexpr double double_min = std::numeric_limits<double>::lowest();
  if ((forwarded_accel_bias_.array() >= double_min).all() &&
      (forwarded_gyro_bias_.array()  >= double_min).all()) {
    maplab_msgs::OdometryWithImuBiases odometry_with_imu_biases;
    odometry_with_imu_biases.header.frame_id =
        frame_names_.refined_frame_corrected;
    odometry_with_imu_biases.header.stamp = current_timestamp;
    odometry_with_imu_biases.child_frame_id = frame_names_.base_link_frame;
    odometry_with_imu_biases.odometry_state = 0u;
    const Transformation refined_odometry = T_L_O_ * T_odom_base;
    tf::poseKindrToMsg(refined_odometry.cast<double>(),
                       &odometry_with_imu_biases.pose.pose);
    for (int row = 0; row < 6; ++row) {
      for (int col = 0; col < 6; ++col) {
        if (row == col) {
          odometry_with_imu_biases.twist.covariance[row * 6 + col] = 1.0;
          odometry_with_imu_biases.pose.covariance[row * 6 + col] = 1.0;
        } else {
          odometry_with_imu_biases.twist.covariance[row * 6 + col] = 0.0;
          odometry_with_imu_biases.pose.covariance[row * 6 + col] = 0.0;
        }
      }
    }
    BiasVectorType zero_vector = BiasVectorType::Zero();
    tf::vectorKindrToMsg(zero_vector,
                         &odometry_with_imu_biases.twist.twist.linear);
    tf::vectorKindrToMsg(zero_vector,
                         &odometry_with_imu_biases.twist.twist.angular);
    tf::vectorKindrToMsg(forwarded_accel_bias_,
                         &odometry_with_imu_biases.accel_bias);
    tf::vectorKindrToMsg(forwarded_gyro_bias_,
                         &odometry_with_imu_biases.gyro_bias);
    odom_with_imu_biases_pub_.publish(odometry_with_imu_biases);
  }

  // Publish the frames
  TfHelper::publishTransform(T_M_L_, frame_names_.mission_frame,
                             frame_names_.refined_frame_corrected, false,
                             current_timestamp);
  TfHelper::publishTransform(T_L_O_, frame_names_.refined_frame_corrected,
                             frame_names_.odom_frame_corrected, false,
                             current_timestamp);
  TfHelper::publishTransform(T_odom_base, frame_names_.odom_frame_corrected,
                             frame_names_.base_link_frame_corrected, false,
                             current_timestamp);
  TfHelper::publishTransform(T_B_C_, frame_names_.base_link_frame_corrected,
                             frame_names_.sensor_frame_corrected, true,
                             current_timestamp);

  // Publish the pose history
  if (pose_history_pub_.getNumSubscribers() > 0) {
    submap_vis_.publishPoseHistory(
        *submap_collection_ptr_, frame_names_.mission_frame, pose_history_pub_);
  }
}

void VoxgraphMapper::imuBiasesCallback(
    const sensor_msgs::Imu::ConstPtr &imu_biases) {
  tf::vectorMsgToKindr(imu_biases->linear_acceleration, &forwarded_accel_bias_);
  tf::vectorMsgToKindr(imu_biases->angular_velocity, &forwarded_gyro_bias_);
  std::cout << "callback: " << imu_biases->angular_velocity.x << std::endl;
}

bool VoxgraphMapper::lookup_T_odom_base(ros::Time timestamp,
                                        Transformation *T_odom_base) {
  CHECK_NOTNULL(T_odom_base);
  Transformation T_odom_base_received;
  double t_waited = 0;   // Total time spent waiting for the updated pose
  double t_max = 0.20;   // Maximum time to wait before giving up
  const ros::Duration timeout(0.005);  // Timeout between each update attempt
  while (t_waited < t_max) {
    if (transformer_.lookupTransform(frame_names_.base_link_frame,
                                     frame_names_.odom_frame, timestamp,
                                     &T_odom_base_received)) {
      *T_odom_base = T_odom_base_received;
      return true;
    }
    timeout.sleep();
    t_waited += timeout.toSec();
  }
  ROS_WARN("Waited %.3fs, but still could not get the TF from %s to %s",
           t_waited, frame_names_.base_link_frame.c_str(),
           frame_names_.odom_frame.c_str());
  return false;
}
}  // namespace voxgraph
