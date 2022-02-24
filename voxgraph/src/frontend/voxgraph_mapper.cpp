#include "voxgraph/frontend/voxgraph_mapper.h"

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/PoseArray.h>
#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_xml.h>
#include <nav_msgs/Path.h>
#include <panoptic_mapping_msgs/SubmapWithPlanes.h>
#include <voxblox_msgs/Mesh.h>
#include <voxblox_msgs/MultiMesh.h>

#include "voxgraph/frontend/plane_collection/plane_type.h"
#include "voxgraph/frontend/plane_collection/submap_stitcher.h"
#include "voxgraph/frontend/submap_collection/submap_timeline.h"
#include "voxgraph/tools/io.h"
#include "voxgraph/tools/ros_params.h"
#include "voxgraph/tools/submap_registration_helper.h"
#include "voxgraph/tools/tf_helper.h"
#include "voxgraph/tools/threading_helper.h"

namespace voxgraph {

VoxgraphMapper::VoxgraphMapper(const ros::NodeHandle& nh,
                               const ros::NodeHandle& nh_private)
    : VoxgraphMapper(nh, nh_private,
                     getVoxgraphSubmapConfigFromRosParams(nh_private),
                     voxblox::getMeshIntegratorConfigFromRosParam(nh_private)) {
}

VoxgraphMapper::VoxgraphMapper(const ros::NodeHandle& nh,
                               const ros::NodeHandle& nh_private,
                               VoxgraphSubmap::Config submap_config,
                               voxblox::MeshIntegratorConfig mesh_config)
    : nh_(nh),
      nh_private_(nh_private),
      verbose_(false),
      auto_pause_rosbag_(false),
      rosbag_helper_(nh),
      submap_pose_tf_publishing_period_s_(0.5),
      loop_closure_topic_("loop_closure_input"),
      submap_topic_("submap_input"),
      submap_with_planes_topic_("submap_with_planes_input"),
      loop_closure_topic_queue_length_(1000),
      submap_topic_queue_length_(10),
      publisher_queue_length_(100),
      odometry_constraints_enabled_(false),
      height_constraints_enabled_(false),
      plane_constraints_enabled_(false),
      pause_sliding_optimization_(false),
      pause_full_optimization_(false),
      submap_config_(submap_config),
      submap_collection_ptr_(
          std::make_shared<VoxgraphSubmapCollection>(submap_config_)),
      submap_vis_(submap_config_, mesh_config),
      full_pose_graph_optimization_period_s_(0.0),
      pose_graph_manager_(
          nh_private, submap_collection_ptr_, mesh_config,
          FrameNames::fromRosParams(nh_private).output_odom_frame),
      projected_map_server_(nh_private),
      submap_server_(nh_private),
      frame_names_(FrameNames::fromRosParams(nh_private)),
      robocentric_robot_name_("") {
  // Setup interaction with ROS
  getParametersFromRos();
  subscribeToTopics();
  advertiseTopics();
  advertiseServices();

  // Setup timers
  if (0.0 < submap_pose_tf_publishing_period_s_) {
    submap_pose_tf_publishing_timer_ = nh_private.createTimer(
        ros::Duration(submap_pose_tf_publishing_period_s_),
        std::bind(&VoxgraphMapper::publishSubmapPoseTFs, this));
  }
  if (0.0 < full_pose_graph_optimization_period_s_) {
    full_pose_graph_optimization_timer_ = nh_private.createTimer(
        ros::Duration(full_pose_graph_optimization_period_s_),
        std::bind(&VoxgraphMapper::optimizeFullPoseGraph, this,
                  /* skip_if_busy */ true));
  }
}

void VoxgraphMapper::getParametersFromRos() {
  nh_private_.param("verbose", verbose_, verbose_);
  pose_graph_manager_.setVerbosity(verbose_);

  nh_private_.param("loop_closure_topic", loop_closure_topic_,
                    loop_closure_topic_);
  nh_private_.param("loop_closure_topic_queue_length",
                    loop_closure_topic_queue_length_,
                    loop_closure_topic_queue_length_);
  nh_private_.param("submap_topic", submap_topic_, submap_topic_);
  nh_private_.param("submap_with_planes_topic", submap_with_planes_topic_,
                    submap_with_planes_topic_);
  nh_private_.param("submap_topic_queue_length", submap_topic_queue_length_,
                    submap_topic_queue_length_);
  nh_private_.param("publisher_queue_length", publisher_queue_length_,
                    publisher_queue_length_);

  // Check whether to set the submap collection poses inertial frame,
  // or in the robocentric frame for a specific robot
  {
    nh_private_.param("robocentric_robot_name", robocentric_robot_name_,
                      robocentric_robot_name_);
    pose_graph_manager_.setRobocentricRobotName(robocentric_robot_name_);
  }

  // Get the submap creation interval as a ros::Duration
  double interval_temp;
  if (nh_private_.getParam("submap_creation_interval", interval_temp)) {
    submap_collection_ptr_->setSubmapCreationInterval(
        ros::Duration(interval_temp));
  }

  submap_vis_.setMeshOpacity(nh_private_.param("mesh_opacity", 1.0));
  submap_vis_.setSubmapMeshColorMode(
      voxblox::getColorModeFromString(nh_private_.param<std::string>(
          "submap_mesh_color_mode", "lambert_color")));
  submap_vis_.setCombinedMeshColorMode(voxblox::getColorModeFromString(
      nh_private_.param<std::string>("combined_mesh_color_mode", "normals")));

  // Read whether or not to auto pause the rosbag during graph optimization
  nh_private_.param("auto_pause_rosbag", auto_pause_rosbag_,
                    auto_pause_rosbag_);

  // Get the submap pose TF update timer period
  nh_private_.param("submap_pose_tf_publishing_period_s",
                    submap_pose_tf_publishing_period_s_,
                    submap_pose_tf_publishing_period_s_);

  // Get the pose graph optimization period
  nh_private_.param("full_pose_graph_optimization_period_s",
                    full_pose_graph_optimization_period_s_,
                    full_pose_graph_optimization_period_s_);

  // Read the measurement params from their sub-namespace
  bool registration_constraints_enabled = false;
  ros::NodeHandle nh_measurement_params(nh_private_, "measurements");
  nh_measurement_params.param("submap_registration/enabled",
                              registration_constraints_enabled,
                              registration_constraints_enabled);
  pose_graph_manager_.setRegistrationConstraintsEnabled(
      registration_constraints_enabled);
  ROS_INFO_STREAM_COND(
      verbose_, "Submap registration constraints: "
                    << (pose_graph_manager_.getRegistrationConstraintsEnabled()
                            ? "enabled"
                            : "disabled"));
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
  nh_measurement_params.param("planes/enabled", plane_constraints_enabled_,
                              plane_constraints_enabled_);
  ROS_INFO_STREAM_COND(
      verbose_, "Planes constraints: "
                    << (plane_constraints_enabled_ ? "enabled" : "disabled"));

  pose_graph_manager_.setMeasurementConfigFromRosParams(nh_measurement_params);
}

void VoxgraphMapper::subscribeToTopics() {
  loop_closure_subscriber_ =
      nh_.subscribe(loop_closure_topic_, loop_closure_topic_queue_length_,
                    &VoxgraphMapper::loopClosureCallback, this);
  submap_subscriber_ = nh_.subscribe<voxblox_msgs::Submap>(
      submap_topic_, submap_topic_queue_length_,
      [this](const voxblox_msgs::Submap::ConstPtr& msg) {
        submapCallback(*msg);
      });
  submap_with_planes_subscriber_ =
      nh_.subscribe<panoptic_mapping_msgs::SubmapWithPlanes>(
          submap_with_planes_topic_, submap_topic_queue_length_,
          [this](const panoptic_mapping_msgs::SubmapWithPlanes::ConstPtr& msg) {
            submapWithPlanesCallback(*msg);
          });
}

void VoxgraphMapper::advertiseTopics() {
  submap_mesh_pub_ = nh_private_.advertise<voxblox_msgs::MultiMesh>(
      "separated_mesh", publisher_queue_length_, true);
  combined_mesh_pub_ = nh_private_.advertise<voxblox_msgs::Mesh>(
      "combined_mesh", publisher_queue_length_, true);
  pose_history_pub_ =
      nh_private_.advertise<nav_msgs::Path>("pose_history", 1, true);
  loop_closure_links_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>(
          "loop_closure_links_vis", publisher_queue_length_, true);
  loop_closure_axes_pub_ = nh_private_.advertise<geometry_msgs::PoseArray>(
      "loop_closure_axes_vis", publisher_queue_length_, true);
}

void VoxgraphMapper::advertiseServices() {
  publish_separated_mesh_srv_ = nh_private_.advertiseService(
      "publish_separated_mesh", &VoxgraphMapper::publishSeparatedMeshCallback,
      this);
  publish_combined_mesh_srv_ = nh_private_.advertiseService(
      "publish_combined_mesh", &VoxgraphMapper::publishCombinedMeshCallback,
      this);
  optimize_graph_srv_ = nh_private_.advertiseService(
      "optimize_pose_graph", &VoxgraphMapper::optimizeGraphCallback, this);
  finish_map_srv_ = nh_private_.advertiseService(
      "finish_map", &VoxgraphMapper::finishMapCallback, this);
  save_to_file_srv_ = nh_private_.advertiseService(
      "save_to_file", &VoxgraphMapper::saveToFileCallback, this);
  save_pose_history_to_file_srv_ = nh_private_.advertiseService(
      "save_pose_history_to_file",
      &VoxgraphMapper::savePoseHistoryToFileCallback, this);
  save_separated_mesh_srv_ = nh_private_.advertiseService(
      "save_separated_mesh", &VoxgraphMapper::saveSeparatedMeshCallback, this);
  save_combined_mesh_srv_ = nh_private_.advertiseService(
      "save_combined_mesh", &VoxgraphMapper::saveCombinedMeshCallback, this);
  save_optimization_times_srv_ = nh_private_.advertiseService(
      "save_optimization_times", &VoxgraphMapper::saveOptimizationTimesCallback,
      this);
  pause_sliding_optimization_srv_ = nh_private_.advertiseService(
      "pause_sliding_optimization",
      &VoxgraphMapper::pauseSlidingOptimizationCallback, this);
  pause_full_optimization_srv_ = nh_private_.advertiseService(
      "pause_full_optimization", &VoxgraphMapper::pauseFullOptimizationCallback,
      this);
}

void VoxgraphMapper::loopClosureCallback(
    const voxgraph_msgs::LoopClosure& loop_closure_msg) {
  ROS_ERROR("Loop closure callback currently unavailable.");
  //  // TODO(victorr): Introduce flag to switch between default or msg info.
  //  matrix
  //  // TODO(victorr): Move the code below to a measurement processor
  //  // Setup warning msg prefix
  //  const ros::Time& timestamp_A = loop_closure_msg.from_timestamp;
  //  const ros::Time& timestamp_B = loop_closure_msg.to_timestamp;
  //  std::ostringstream warning_msg_prefix;
  //  warning_msg_prefix << "Could not add loop closure from timestamp "
  //                     << timestamp_A << " to " << timestamp_B;
  //
  //  // Find the submaps that were active at both timestamps
  //  SubmapID submap_id_A, submap_id_B;
  //  bool success_A = submap_collection_ptr_->lookupActiveSubmapByTime(
  //      loop_closure_msg.from_timestamp, &submap_id_A);
  //  bool success_B = submap_collection_ptr_->lookupActiveSubmapByTime(
  //      loop_closure_msg.to_timestamp, &submap_id_B);
  //  if (!success_A || !success_B) {
  //    ROS_WARN_STREAM(warning_msg_prefix.str() << ": timestamp A or B has no "
  //                                                "corresponding submap");
  //    return;
  //  }
  //  if (submap_id_A == submap_id_B) {
  //    ROS_WARN_STREAM(warning_msg_prefix.str() << ": timestamp A and B fall "
  //                                                "within the same submap");
  //    return;
  //  }
  //  const VoxgraphSubmap& submap_A =
  //      submap_collection_ptr_->getSubmap(submap_id_A);
  //  const VoxgraphSubmap& submap_B =
  //      submap_collection_ptr_->getSubmap(submap_id_B);
  //
  //  // Find the robot pose at both timestamp in their active submap frame
  //  Transformation T_A_t1, T_B_t2;
  //  if (!submap_A.lookupPoseByTime(timestamp_A, &T_A_t1) ||
  //      !submap_B.lookupPoseByTime(timestamp_B, &T_B_t2)) {
  //    ROS_WARN_STREAM(warning_msg_prefix.str() << ": timestamp A or B has no "
  //                                                "corresponding robot pose");
  //    return;
  //  }
  //
  //  // Convert the transform between two timestamps into a transform between
  //  // two submap origins
  //  // NOTE: The usual conversion method, tf::transformMsgToKindr(), is not
  //  used
  //  //       since it has a hard CHECK on invalid rotations which cannot be
  //  caught Eigen::Vector3d translation;
  //  tf::vectorMsgToEigen(loop_closure_msg.transform.translation, translation);
  //  Eigen::Quaterniond rotation;
  //  tf::quaternionMsgToEigen(loop_closure_msg.transform.rotation, rotation);
  //  if (std::abs(rotation.squaredNorm() - 1.0) > 1e-3) {
  //    ROS_WARN_STREAM(warning_msg_prefix.str() << ": supplied transform "
  //                                                "quaternion is invalid");
  //    return;
  //  }
  //  Transformation T_t1_t2(translation.cast<voxblox::FloatingPoint>(),
  //                         rotation.cast<voxblox::FloatingPoint>());
  //  Transformation T_AB = T_A_t1 * T_t1_t2 * T_B_t2.inverse();
  //  pose_graph_manager_.addLoopClosureMeasurement(submap_id_A, submap_id_B,
  //  T_AB);
  //
  //  // Visualize the loop closure link
  //  const Transformation& T_O_A = submap_A.getPose();
  //  const Transformation& T_O_B = submap_B.getPose();
  //  const Transformation T_O_t1 = T_O_A * T_A_t1;
  //  const Transformation T_O_t2 = T_O_B * T_B_t2;
  //  loop_closure_vis_.publishLoopClosure(T_O_t1, T_O_t2, T_t1_t2,
  //                                       frame_names_.output_odom_frame,
  //                                       loop_closure_links_pub_);
  //  loop_closure_vis_.publishAxes(T_O_t1, T_O_t2, T_t1_t2,
  //                                frame_names_.output_odom_frame,
  //                                loop_closure_axes_pub_);
}

SubmapID VoxgraphMapper::submapCallback(
    const voxblox_msgs::Submap& submap_msg) {
  // Create the new submap draft
  VoxgraphSubmap new_submap = submap_collection_ptr_->draftNewSubmap();

  // Deserialize the submap trajectory
  std::string mismatched_odom_frame;
  if (submap_msg.trajectory.poses.empty()) {
    ROS_WARN("Received submap with empty trajectory. Skipping submap.");
    return kInvalidSubmapId;
  }
  for (const geometry_msgs::PoseStamped& pose_stamped :
       submap_msg.trajectory.poses) {
    if (frame_names_.input_odom_frame != pose_stamped.header.frame_id) {
      mismatched_odom_frame = pose_stamped.header.frame_id;
    }
    TransformationD T_odom_base_link;
    tf::poseMsgToKindr(pose_stamped.pose, &T_odom_base_link);
    new_submap.addPoseToHistory(
        pose_stamped.header.stamp,
        T_odom_base_link.cast<voxblox::FloatingPoint>());
    all_poses_history_.push_back(pose_stamped);
  }
  ROS_WARN_STREAM_COND(!mismatched_odom_frame.empty(),
                       "All submap trajectory poses are expected in frame "
                           << frame_names_.input_odom_frame
                           << ", but encountered pose in frame "
                           << mismatched_odom_frame << " instead.");

  // Deserialize the submap TSDF
  if (!voxblox::deserializeMsgToLayer(
          submap_msg.layer, new_submap.getTsdfMapPtr()->getTsdfLayerPtr())) {
    ROS_WARN("Received a submap msg with an invalid TSDF. Skipping submap.");
    return kInvalidSubmapId;
  }

  // Set the robot name
  new_submap.setRobotName(submap_msg.robot_name);

  // The submap pose corresponds to its origin, which is the origin of the
  // submap_msg
  // NOTE: We will implicitly update the submap pose later on, when moving the
  //       submap origin to the middle pose of the trajectory
  const Transformation T_O_O = Transformation();
  new_submap.setPose(T_O_O);  // Identity transform

  // Transform the submap from odom to submap frame
  const size_t trajectory_middle_idx = new_submap.getPoseHistory().size() / 2;
  TransformationD T_odom_trajectory_middle_pose;
  tf::poseMsgToKindr(submap_msg.trajectory.poses[trajectory_middle_idx].pose,
                     &T_odom_trajectory_middle_pose);
  const Transformation T_odom_submap =
      VoxgraphSubmapCollection::gravityAlignPose(
          T_odom_trajectory_middle_pose.cast<voxblox::FloatingPoint>());
  new_submap.transformSubmap(T_odom_submap.inverse());
  // NOTE: In addition to changing the origin for the submap's trajectory and
  //       TSDF, it will also update the submap's T_O_S pose (such that the
  //       voxels don't move with respect to the odom frame) and update all
  //       cached members including the ESDF (implicitly creating it).

  // Add finished new submap to the submap collection
  submap_collection_ptr_->addSubmap(std::move(new_submap));

  // Add the new submap to the pose graph
  SubmapID active_submap_id = submap_collection_ptr_->getActiveSubmapID();
  pose_graph_manager_.addSubmap(active_submap_id);

  // Add an odometry constraint from the previous to the new submap
  SubmapID previous_submap_id;
  if (odometry_constraints_enabled_ &&
      submap_collection_ptr_->getPreviousSubmapId(submap_msg.robot_name,
                                                  &previous_submap_id)) {
    // Compute the odometry
    const VoxgraphSubmap& previous_submap =
        submap_collection_ptr_->getSubmap(previous_submap_id);
    const Transformation T_O_previous_submap = previous_submap.getInitialPose();
    const Transformation T_previous_current_submap =
        T_O_previous_submap.inverse() * T_odom_submap;

    // Add the constraint to the pose graph
    pose_graph_manager_.addOdometryMeasurement(
        previous_submap_id, active_submap_id, T_previous_current_submap);
  }

  // Add a height constraint if appropriate
  if (height_constraints_enabled_) {
    // This assumes that the height estimate from the odometry source is
    // really good (e.g. when it fuses an altimeter)
    double current_height = T_odom_submap.getPosition().z();
    pose_graph_manager_.addHeightMeasurement(active_submap_id, current_height);
  }

  // NOTE: Registration constraints are managed in the graph optimization calls

  // Align the new submap to the collection
  optimizeSlidingPoseGraph();

  // Launch a new full pose graph optimization if the previous one finished,
  // if we're running the optimization as fast as possible instead of on a timer
  if (full_pose_graph_optimization_period_s_ <= 0.0) {
    optimizeFullPoseGraph(/* skip_if_busy */ true);
  }

  // Publish the map in its different representations
  ros::Time latest_timestamp = submap_msg.trajectory.poses.back().header.stamp;
  publishMaps(latest_timestamp);
  publishSubmapPoseTFs();

  // Signal that the new submap was successfully added
  return active_submap_id;
}

SubmapID VoxgraphMapper::submapWithPlanesCallback(
    const panoptic_mapping_msgs::SubmapWithPlanes& submap_planes_msg) {
  const voxblox_msgs::Submap& submap_msg = submap_planes_msg.submap;
  // Create the new submap draft
  VoxgraphSubmap new_submap = submap_collection_ptr_->draftNewSubmap();

  // Deserialize the submap trajectory
  std::string mismatched_odom_frame;
  if (submap_msg.trajectory.poses.empty()) {
    ROS_WARN("Received submap with empty trajectory. Skipping submap.");
    return kInvalidSubmapId;
  }
  for (const geometry_msgs::PoseStamped& pose_stamped :
       submap_msg.trajectory.poses) {
    if (frame_names_.input_odom_frame != pose_stamped.header.frame_id) {
      mismatched_odom_frame = pose_stamped.header.frame_id;
    }
    TransformationD T_odom_base_link;
    tf::poseMsgToKindr(pose_stamped.pose, &T_odom_base_link);
    new_submap.addPoseToHistory(
        pose_stamped.header.stamp,
        T_odom_base_link.cast<voxblox::FloatingPoint>());
    all_poses_history_.push_back(pose_stamped);
  }
  ROS_WARN_STREAM_COND(!mismatched_odom_frame.empty(),
                       "All submap trajectory poses are expected in frame "
                           << frame_names_.input_odom_frame
                           << ", but encountered pose in frame "
                           << mismatched_odom_frame << " instead.");

  // Deserialize the submap TSDF
  if (!voxblox::deserializeMsgToLayer(
          submap_msg.layer, new_submap.getTsdfMapPtr()->getTsdfLayerPtr())) {
    ROS_WARN("Received a submap msg with an invalid TSDF. Skipping submap.");
    return kInvalidSubmapId;
  }

  // Set the robot name
  new_submap.setRobotName(submap_msg.robot_name);

  // The submap pose corresponds to its origin, which is the origin of the
  // submap_msg
  // NOTE: We will implicitly update the submap pose later on, when moving the
  //       submap origin to the middle pose of the trajectory
  const Transformation T_O_O = Transformation();
  new_submap.setPose(T_O_O);  // Identity transform

  // Transform the submap from odom to submap frame
  const size_t trajectory_middle_idx = new_submap.getPoseHistory().size() / 2;
  TransformationD T_odom_trajectory_middle_pose;
  tf::poseMsgToKindr(submap_msg.trajectory.poses[trajectory_middle_idx].pose,
                     &T_odom_trajectory_middle_pose);
  const Transformation T_odom_submap =
      VoxgraphSubmapCollection::gravityAlignPose(
          T_odom_trajectory_middle_pose.cast<voxblox::FloatingPoint>());
  new_submap.transformSubmap(T_odom_submap.inverse());
  // NOTE: In addition to changing the origin for the submap's trajectory and
  //       TSDF, it will also update the submap's T_O_S pose (such that the
  //       voxels don't move with respect to the odom frame) and update all
  //       cached members including the ESDF (implicitly creating it).

  // Add finished new submap to the submap collection
  submap_collection_ptr_->addSubmap(std::move(new_submap));

  // Add the new submap to the pose graph
  SubmapID active_submap_id = submap_collection_ptr_->getActiveSubmapID();
  pose_graph_manager_.addSubmap(active_submap_id);

  // Add an odometry constraint from the previous to the new submap
  SubmapID previous_submap_id;
  if (odometry_constraints_enabled_ &&
      submap_collection_ptr_->getPreviousSubmapId(submap_msg.robot_name,
                                                  &previous_submap_id)) {
    // Compute the odometry
    const VoxgraphSubmap& previous_submap =
        submap_collection_ptr_->getSubmap(previous_submap_id);
    const Transformation T_O_previous_submap = previous_submap.getInitialPose();
    const Transformation T_previous_current_submap =
        T_O_previous_submap.inverse() * T_odom_submap;

    // Add the constraint to the pose graph
    pose_graph_manager_.addOdometryMeasurement(
        previous_submap_id, active_submap_id, T_previous_current_submap);
  }

  // Add a height constraint if appropriate
  if (height_constraints_enabled_) {
    // This assumes that the height estimate from the odometry source is
    // really good (e.g. when it fuses an altimeter)
    double current_height = T_odom_submap.getPosition().z();
    pose_graph_manager_.addHeightMeasurement(active_submap_id, current_height);
  }

  if (plane_constraints_enabled_) {
    LOG(ERROR) << "plane_constraints_enabled_ youhou!!!";
    // get classes to planes from message
    classToPlanesType classes_to_planes;
    for (const auto& plane_msg : submap_planes_msg.planes) {
      const PlaneType plane = PlaneType::fromMsg(plane_msg);
      int class_id = plane_msg.class_id;
      LOG(ERROR) << "Adding plane with class_id:" << class_id
                 << "at point:\n" << plane.getPointInit() << "\nnormal:\n"
                 << plane.getPlaneNormal();
      if (classes_to_planes.find(class_id) != classes_to_planes.end()) {
        classes_to_planes.at(class_id).push_back(plane);
      } else {
        classes_to_planes.insert({class_id, std::vector<PlaneType>{plane}});
      }
    }
    // find all planes of the current submap that match with previously sent
    // planes of submaps
    std::map<int, int> matched_planes;
    std::vector<int> submap_ids;
    submap_stitcher_.addSubmapPlanes(new_submap.getID(), classes_to_planes);
    submap_stitcher_.findAllPlaneMatchesForSubmap(
        new_submap, submap_collection_ptr_->getSubmapConstPtrs(),
        &matched_planes, &submap_ids);
    // construct constraint
    LOG(ERROR) << "Added " << matched_planes.size() << " plane matches";
    pose_graph_manager_.addPlanesMeasurement(new_submap.getID(), submap_ids,
                                             matched_planes,
                                             submap_stitcher_.getAllPlanes());
    LOG(ERROR) << "plane constraint constructed!";
  } else {
    LOG(ERROR) << "plane_constraints_NOT_enabled_ Bouhouhou!!!";
  }
  // NOTE: Registration constraints are managed in the graph optimization calls

  // Align the new submap to the collection
  optimizeSlidingPoseGraph();

  // Launch a new full pose graph optimization if the previous one finished,
  // if we're running the optimization as fast as possible instead of on a timer
  if (full_pose_graph_optimization_period_s_ <= 0.0) {
    optimizeFullPoseGraph(/* skip_if_busy */ true);
  }

  // Publish the map in its different representations
  ros::Time latest_timestamp = submap_msg.trajectory.poses.back().header.stamp;
  publishMaps(latest_timestamp);
  publishSubmapPoseTFs();

  // Signal that the new submap was successfully added
  return active_submap_id;
}

bool VoxgraphMapper::publishSeparatedMeshCallback(
    std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  for (const VoxgraphSubmap::ConstPtr& submap_ptr :
       submap_collection_ptr_->getSubmapConstPtrs()) {
    submap_vis_.publishMesh(*submap_collection_ptr_, submap_ptr->getID(),
                            "submap_" + std::to_string(submap_ptr->getID()),
                            submap_mesh_pub_);
  }

  return true;  // Tell ROS it succeeded
}

bool VoxgraphMapper::publishCombinedMeshCallback(
    std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  submap_vis_.publishCombinedMesh(*submap_collection_ptr_,
                                  frame_names_.output_odom_frame,
                                  combined_mesh_pub_);
  return true;  // Tell ROS it succeeded
}

bool VoxgraphMapper::finishMapCallback(std_srvs::Empty::Request& request,
                                       std_srvs::Empty::Response& response) {
  // Indicate that the previous submap is finished
  // s.t. its cached members are generated
  if (!submap_collection_ptr_->empty()) {
    ROS_INFO("Finishing the last submap");
    submap_collection_ptr_->getActiveSubmapPtr()->finishSubmap();
  }

  // Optimize the pose graph
  optimizeFullPoseGraph(/* skip_if_busy */ false);

  // Publishing the finished submap
  submap_server_.publishActiveSubmap(submap_collection_ptr_, ros::Time::now());
  publishMaps(ros::Time::now());
  io::savePoseHistoryToFile("/home/ioannis/datasets/vox_input.bag",
                            all_poses_history_);
  ROS_INFO("The map is now finished and ready to be saved");
  return true;
}

bool VoxgraphMapper::optimizeGraphCallback(
    std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  optimizeFullPoseGraph(/* skip_if_busy */ false);
  return true;
}

bool VoxgraphMapper::saveToFileCallback(
    voxblox_msgs::FilePath::Request& request,
    voxblox_msgs::FilePath::Response& response) {
  submap_collection_ptr_->saveToFile(request.file_path);
  return true;
}

bool VoxgraphMapper::savePoseHistoryToFileCallback(
    voxblox_msgs::FilePath::Request& request,
    voxblox_msgs::FilePath::Response& response) {
  ROS_INFO_STREAM("Writing pose history to bag at: " << request.file_path);
  io::savePoseHistoryToFile(request.file_path,
                            submap_collection_ptr_->getPoseHistory());
  return true;
}

bool VoxgraphMapper::saveSeparatedMeshCallback(
    voxblox_msgs::FilePath::Request& request,
    voxblox_msgs::FilePath::Response& response) {
  submap_vis_.saveSeparatedMesh(request.file_path, *submap_collection_ptr_);
  return true;  // Tell ROS it succeeded
}

bool VoxgraphMapper::saveCombinedMeshCallback(
    voxblox_msgs::FilePath::Request& request,
    voxblox_msgs::FilePath::Response& response) {
  submap_vis_.saveCombinedMesh(request.file_path, *submap_collection_ptr_);
  return true;  // Tell ROS it succeeded
}

bool VoxgraphMapper::saveOptimizationTimesCallback(
    voxblox_msgs::FilePath::Request& request,
    voxblox_msgs::FilePath::Response& response) {
  ROS_INFO_STREAM(
      "Writing optimization times to csv at: " << request.file_path);
  const PoseGraph::SolverSummaryList& solver_summary_list =
      pose_graph_manager_.getFullPoseGraphSolverSummaries();
  std::vector<double> total_times;
  for (const ceres::Solver::Summary& summary : solver_summary_list) {
    total_times.push_back(summary.total_time_in_seconds);
  }
  io::saveVectorToFile(request.file_path, total_times);
  return true;  // Tell ROS it succeeded
}

bool VoxgraphMapper::pauseSlidingOptimizationCallback(
    std_srvs::SetBool::Request& request,
    std_srvs::SetBool::Response& response) {
  pause_sliding_optimization_ = request.data;
  response.success = true;
  return true;
}

bool VoxgraphMapper::pauseFullOptimizationCallback(
    std_srvs::SetBool::Request& request,
    std_srvs::SetBool::Response& response) {
  pause_full_optimization_ = request.data;
  response.success = true;
  return true;
}

int VoxgraphMapper::optimizeSlidingPoseGraph() {
  // Avoid optimizing an empty pose graph
  if (submap_collection_ptr_->size() < 2) {
    return 1;
  }

  if (!pause_sliding_optimization_) {
    // Redetect and add the relevant registration constraints
    pose_graph_manager_.updateSlidingPoseGraphRegistrationConstraints();

    // Optimize the pose graph
    ROS_INFO_STREAM("Optimizing the sliding pose graph");
    pose_graph_manager_.optimizeSlidingPoseGraph();
  }

  // Publish updated poses
  submap_server_.publishSubmapPoses(submap_collection_ptr_, ros::Time::now());

  // Report successful completion
  return 1;
}

int VoxgraphMapper::optimizeFullPoseGraph(const bool skip_if_busy) {
  // Avoid optimizing an empty pose graph
  if (submap_collection_ptr_->size() < 2) {
    return 1;
  }

  // Wait for the last optimization to finish before updating the constraints
  if (optimization_async_handle_.valid() &&
      optimization_async_handle_.wait_for(std::chrono::milliseconds(10)) !=
          std::future_status::ready) {
    if (skip_if_busy) {
      ROS_INFO(
          "Previous full pose graph optimization not yet complete. "
          "Skipping...");
      return 0;
    } else {
      ROS_WARN(
          "Previous full pose graph optimization not yet complete. Waiting...");
      optimization_async_handle_.wait();
    }
  }

  // Move the new sliding window nodes and constraints to the full pose graph
  ROS_INFO("Absorbing sliding window into full pose graph");
  pose_graph_manager_.absorbSlidingWindowIntoFullPoseGraph();

  if (!pause_full_optimization_) {
    // Redetect and add the relevant registration constraints
    pose_graph_manager_.updateFullPoseGraphRegistrationConstraints();

    // Optimize the pose graph
    optimization_async_handle_ = std::async(std::launch::async, [this]() {
      ROS_INFO("Starting full pose graph optimization in separate thread");
      pose_graph_manager_.optimizeFullPoseGraph();

      // Publish the updated poses
      submap_server_.publishSubmapPoses(submap_collection_ptr_,
                                        ros::Time::now());
    });
  }

  // Report successful completion
  return 1;
}

void VoxgraphMapper::publishMaps(const ros::Time& current_timestamp) {
  // Publish the submap poses as msgs
  submap_server_.publishSubmapPoses(submap_collection_ptr_, current_timestamp);

  // Publish the projected map as a mesh
  if (0 < combined_mesh_pub_.getNumSubscribers()) {
    // NOTE: Users can request new meshes at any time through service calls
    //       so there's no point in publishing them just in case
    ThreadingHelper::launchBackgroundThread(
        &SubmapVisuals::publishCombinedMesh, &submap_vis_,
        *submap_collection_ptr_, frame_names_.output_odom_frame,
        combined_mesh_pub_);
  }

  // Publish the projected map as a TSDF
  projected_map_server_.publishProjectedMap(*submap_collection_ptr_,
                                            current_timestamp);

  // Publish the new submap's TSDF, ESDF, and surface vertices
  if (!submap_collection_ptr_->empty()) {
    submap_server_.publishSubmap(submap_collection_ptr_->getActiveSubmap(),
                                 current_timestamp);
    if (0 < submap_mesh_pub_.getNumSubscribers()) {
      const SubmapID active_submap_id =
          submap_collection_ptr_->getActiveSubmapID();
      submap_vis_.publishMesh(*submap_collection_ptr_, active_submap_id,
                              robocentric_robot_name_ + "_submap_" +
                                  std::to_string(active_submap_id),
                              submap_mesh_pub_);
    }
  }

  // Publish the robot trajectory
  if (0 < pose_history_pub_.getNumSubscribers()) {
    submap_vis_.publishPoseHistory(*submap_collection_ptr_,
                                   frame_names_.output_odom_frame,
                                   pose_history_pub_);
  }
}

void VoxgraphMapper::publishSubmapPoseTFs() {
  // Publish the submap poses as TFs
  const ros::Time current_timestamp = ros::Time::now();
  // NOTE: The rest of the voxgraph is purely driven by the sensor message
  //       timestamps, but the submap pose TFs are published at the current ROS
  //       time (e.g. computer time or /clock topic if use_time_time:=true).
  //       This is necessary because TF lookups only interpolate (i.e. no
  //       extrapolation), and TFs subscribers typically have a limited buffer
  //       time length (e.g. Rviz). The TFs therefore have to be published at a
  //       frequency that exceeds the rate at which new submaps come in (e.g.
  //       once every 10s). In case you intend to use the TFs for more than
  //       visualization, it would be important that the message timestamps and
  //       ros::Time::now() are well synchronized.
  for (const VoxgraphSubmap::ConstPtr& submap_ptr :
       submap_collection_ptr_->getSubmapConstPtrs()) {
    TfHelper::publishTransform(submap_ptr->getPose(),
                               frame_names_.output_odom_frame,
                               robocentric_robot_name_ + "_submap_" +
                                   std::to_string(submap_ptr->getID()),
                               false, current_timestamp);
  }
  SubmapID first_submap_id;
  // TODO(victorr): Instead of the first submap, use the graph's fixed submap
  if (submap_collection_ptr_->getFirstSubmapId(&first_submap_id)) {
    const VoxgraphSubmap& first_submap =
        submap_collection_ptr_->getSubmap(first_submap_id);
    if (!first_submap.getPoseHistory().empty()) {
      const Transformation T_odom_initial_pose =
          first_submap.getPose() *
          first_submap.getPoseHistory().begin()->second;
      TfHelper::publishTransform(
          T_odom_initial_pose, frame_names_.output_odom_frame,
          robocentric_robot_name_ + "_initial_pose", false, current_timestamp);
    }
  }

  // Restart the timer to avoid queued calls executing at same ROS time
  submap_pose_tf_publishing_timer_.stop();
  submap_pose_tf_publishing_timer_.start();
}
}  // namespace voxgraph
