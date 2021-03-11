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
#include <voxblox_msgs/Mesh.h>
#include <voxblox_msgs/MultiMesh.h>

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
      submap_pose_tf_publishing_period_s_(0.1),
      loop_closure_topic_("loop_closure_input"),
      submap_topic_("submap_input"),
      loop_closure_topic_queue_length_(1000),
      submap_topic_queue_length_(10),
      publisher_queue_length_(100),
      registration_constraints_enabled_(false),
      odometry_constraints_enabled_(false),
      height_constraints_enabled_(false),
      pause_optimization_(false),
      submap_config_(submap_config),
      submap_collection_ptr_(
          std::make_shared<VoxgraphSubmapCollection>(submap_config_)),
      submap_vis_(submap_config_, mesh_config),
      pose_graph_interface_(
          nh_private, submap_collection_ptr_, mesh_config,
          FrameNames::fromRosParams(nh_private).output_odom_frame),
      projected_map_server_(nh_private),
      submap_server_(nh_private),
      loop_closure_edge_server_(nh_private),
      frame_names_(FrameNames::fromRosParams(nh_private)) {
  // Setup interaction with ROS
  getParametersFromRos();
  subscribeToTopics();
  advertiseTopics();
  advertiseServices();
  submap_pose_tf_publishing_timer_ = nh_private.createTimer(
      ros::Duration(submap_pose_tf_publishing_period_s_),
      std::bind(&VoxgraphMapper::publishSubmapPoseTFs, this));
}

void VoxgraphMapper::getParametersFromRos() {
  nh_private_.param("verbose", verbose_, verbose_);
  pose_graph_interface_.setVerbosity(verbose_);

  nh_private_.param("loop_closure_topic", loop_closure_topic_,
                    loop_closure_topic_);
  nh_private_.param("loop_closure_topic_queue_length",
                    loop_closure_topic_queue_length_,
                    loop_closure_topic_queue_length_);
  nh_private_.param("submap_topic", submap_topic_, submap_topic_);
  nh_private_.param("submap_topic_queue_length", submap_topic_queue_length_,
                    submap_topic_queue_length_);
  nh_private_.param("publisher_queue_length", publisher_queue_length_,
                    publisher_queue_length_);

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
}

void VoxgraphMapper::subscribeToTopics() {
  loop_closure_subscriber_ =
      nh_.subscribe(loop_closure_topic_, loop_closure_topic_queue_length_,
                    &VoxgraphMapper::loopClosureCallback, this);
  submap_subscriber_ = nh_.subscribe<voxblox_msgs::LayerWithTrajectory>(
      submap_topic_, submap_topic_queue_length_,
      [this](const voxblox_msgs::LayerWithTrajectory::ConstPtr& msg) {
        submapCallback(*msg);
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
  pause_optimization_srv_ = nh_private_.advertiseService(
      "pause_optimization", &VoxgraphMapper::pauseOptimizationCallback, this);
}

void VoxgraphMapper::loopClosureCallback(
    const voxgraph_msgs::LoopClosure& loop_closure_msg) {
  // TODO(victorr): Introduce flag to switch between default or msg info. matrix
  // TODO(victorr): Move the code below to a measurement processor
  // Setup warning msg prefix
  const ros::Time& timestamp_A = loop_closure_msg.from_timestamp;
  const ros::Time& timestamp_B = loop_closure_msg.to_timestamp;
  std::ostringstream warning_msg_prefix;
  warning_msg_prefix << "Could not add loop closure from timestamp "
                     << timestamp_A << " to " << timestamp_B;

  // Find the submaps that were active at both timestamps
  SubmapID submap_id_A, submap_id_B;
  bool success_A = submap_collection_ptr_->lookupActiveSubmapByTime(
      loop_closure_msg.from_timestamp, &submap_id_A);
  bool success_B = submap_collection_ptr_->lookupActiveSubmapByTime(
      loop_closure_msg.to_timestamp, &submap_id_B);
  if (!success_A || !success_B) {
    ROS_WARN_STREAM(warning_msg_prefix.str() << ": timestamp A or B has no "
                                                "corresponding submap");
    return;
  }
  if (submap_id_A == submap_id_B) {
    ROS_WARN_STREAM(warning_msg_prefix.str() << ": timestamp A and B fall "
                                                "within the same submap");
    return;
  }
  const VoxgraphSubmap& submap_A =
      submap_collection_ptr_->getSubmap(submap_id_A);
  const VoxgraphSubmap& submap_B =
      submap_collection_ptr_->getSubmap(submap_id_B);

  // Find the robot pose at both timestamp in their active submap frame
  Transformation T_A_t1, T_B_t2;
  if (!submap_A.lookupPoseByTime(timestamp_A, &T_A_t1) ||
      !submap_B.lookupPoseByTime(timestamp_B, &T_B_t2)) {
    ROS_WARN_STREAM(warning_msg_prefix.str() << ": timestamp A or B has no "
                                                "corresponding robot pose");
    return;
  }

  // Convert the transform between two timestamps into a transform between
  // two submap origins
  // NOTE: The usual conversion method, tf::transformMsgToKindr(), is not used
  //       since it has a hard CHECK on invalid rotations which cannot be caught
  Eigen::Vector3d translation;
  tf::vectorMsgToEigen(loop_closure_msg.transform.translation, translation);
  Eigen::Quaterniond rotation;
  tf::quaternionMsgToEigen(loop_closure_msg.transform.rotation, rotation);
  if (std::abs(rotation.squaredNorm() - 1.0) > 1e-3) {
    ROS_WARN_STREAM(warning_msg_prefix.str() << ": supplied transform "
                                                "quaternion is invalid");
    return;
  }
  Transformation T_t1_t2(translation.cast<voxblox::FloatingPoint>(),
                         rotation.cast<voxblox::FloatingPoint>());
  Transformation T_AB = T_A_t1 * T_t1_t2 * T_B_t2.inverse();
  pose_graph_interface_.addLoopClosureMeasurement(submap_id_A, submap_id_B,
                                                  T_AB);

  // Visualize the loop closure link
  const Transformation& T_O_A = submap_A.getPose();
  const Transformation& T_O_B = submap_B.getPose();
  const Transformation T_O_t1 = T_O_A * T_A_t1;
  const Transformation T_O_t2 = T_O_B * T_B_t2;
  loop_closure_vis_.publishLoopClosure(T_O_t1, T_O_t2, T_t1_t2,
                                       frame_names_.output_odom_frame,
                                       loop_closure_links_pub_);
  loop_closure_vis_.publishAxes(T_O_t1, T_O_t2, T_t1_t2,
                                frame_names_.output_odom_frame,
                                loop_closure_axes_pub_);
}

bool VoxgraphMapper::submapCallback(
    const voxblox_msgs::LayerWithTrajectory& submap_msg) {
  // Create the new submap draft
  VoxgraphSubmap new_submap = submap_collection_ptr_->draftNewSubmap();

  // Deserialize the submap trajectory
  // TODO(victorr): Add check to ensure that the odom frames from voxblox and
  //                voxgraph match
  if (submap_msg.trajectory.poses.empty()) {
    ROS_WARN("Received submap with empty trajectory. Skipping submap.");
    return false;
  }
  for (const geometry_msgs::PoseStamped& pose_stamped :
       submap_msg.trajectory.poses) {
    TransformationD T_odom_base_link;
    tf::poseMsgToKindr(pose_stamped.pose, &T_odom_base_link);
    new_submap.addPoseToHistory(
        pose_stamped.header.stamp,
        T_odom_base_link.cast<voxblox::FloatingPoint>());
  }

  // Deserialize the submap TSDF
  if (!voxblox::deserializeMsgToLayer(
          submap_msg.layer, new_submap.getTsdfMapPtr()->getTsdfLayerPtr())) {
    ROS_WARN("Received a submap msg with an invalid TSDF. Skipping submap.");
    return false;
  }

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

  // Wait for the last optimization to finish before updating the constraints
  if (optimization_async_handle_.valid() &&
      optimization_async_handle_.wait_for(std::chrono::milliseconds(10)) !=
          std::future_status::ready) {
    ROS_WARN("Previous pose graph optimization not yet complete. Waiting...");
    optimization_async_handle_.wait();
  }

  // Add the new submap to the pose graph
  SubmapID active_submap_id = submap_collection_ptr_->getActiveSubmapID();
  pose_graph_interface_.addSubmap(active_submap_id);

  if (submap_collection_ptr_->size() > 1) {
    // Get the previous submap's optimized pose
    SubmapID previous_submap_id = submap_collection_ptr_->getPreviousSubmapId();
    Transformation T_odom__previous_submap;
    submap_collection_ptr_->getSubmapPose(previous_submap_id,
                                          &T_odom__previous_submap);

    // Compute the odometry
    const Transformation T_previous_submap__submap =
        T_odom__previous_submap.inverse() * T_odom_submap;

    // Add an odometry constraint from the previous to the new submap
    if (odometry_constraints_enabled_) {
      // Add the constraint to the pose graph
      pose_graph_interface_.addOdometryMeasurement(
          submap_collection_ptr_->getPreviousSubmapId(),
          submap_collection_ptr_->getActiveSubmapID(),
          T_previous_submap__submap);
    }

    // Constrain the height
    // NOTE: No constraint is added for the 1st submap since its pose is fixed
    if (height_constraints_enabled_) {
      // This assumes that the height estimate from the odometry source is
      // really good (e.g. when it fuses an altimeter)
      double current_height = T_odom_submap.getPosition().z();
      pose_graph_interface_.addHeightMeasurement(active_submap_id,
                                                 current_height);
    }
  } else {
    // Since we did not yet perform any optimizations, the odom and odom
    // frame are equal
    // NOTE: The pose has already implicitly been set to T_odom_submap,
    //       by the earlier transformSubmap(...) call.
    //       We just do it again to keep the code easy to follow.
    submap_collection_ptr_->getActiveSubmapPtr()->setPose(T_odom_submap);
  }

  // Add registration constraints for all overlapping submaps
  if (registration_constraints_enabled_) {
    pose_graph_interface_.updateRegistrationConstraints();
  }

  // Optimize the pose graph in a separate thread
  if (!pause_optimization_) {
    optimization_async_handle_ = std::async(
        std::launch::async, &VoxgraphMapper::optimizePoseGraph, this);
  }

  // Publish the map in its different representations
  ros::Time latest_timestamp = submap_msg.trajectory.poses.back().header.stamp;
  publishMaps(latest_timestamp);
  publishSubmapPoseTFs();

  // Signal that the new submap was successfully added
  return true;
}

bool VoxgraphMapper::publishSeparatedMeshCallback(
    std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  submap_vis_.publishSeparatedMesh(*submap_collection_ptr_,
                                   frame_names_.output_odom_frame,
                                   submap_mesh_pub_);
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

  // Add registration constraints for all overlapping submaps
  if (registration_constraints_enabled_) {
    ROS_INFO(
        "Updating the registration constraints "
        "to include the last submap");
    pose_graph_interface_.updateRegistrationConstraints();
  }

  // Optimize the pose graph
  optimizePoseGraph();

  // Publishing the finished submap
  submap_server_.publishActiveSubmap(submap_collection_ptr_, ros::Time::now());
  publishMaps(ros::Time::now());

  ROS_INFO("The map is now finished and ready to be saved");
  return true;
}

bool VoxgraphMapper::optimizeGraphCallback(
    std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  optimizePoseGraph();
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
      pose_graph_interface_.getSolverSummaries();
  std::vector<double> total_times;
  for (const ceres::Solver::Summary& summary : solver_summary_list) {
    total_times.push_back(summary.total_time_in_seconds);
  }
  io::saveVectorToFile(request.file_path, total_times);
  return true;  // Tell ROS it succeeded
}

bool VoxgraphMapper::pauseOptimizationCallback(
    std_srvs::SetBool::Request& request,
    std_srvs::SetBool::Response& response) {
  pause_optimization_ = request.data;
  response.success = true;
  return true;
}

int VoxgraphMapper::optimizePoseGraph() {
  // Optimize the pose graph
  ROS_INFO("Optimizing the pose graph");
  pose_graph_interface_.optimize();

  // Update the submap poses
  pose_graph_interface_.updateSubmapCollectionPoses();

  // Publish updated poses
  submap_server_.publishSubmapPoses(submap_collection_ptr_, ros::Time::now());

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
                              "submap_" + std::to_string(active_submap_id),
                              submap_mesh_pub_);
    }
  }

  // Publish the robot trajectory
  if (0 < pose_history_pub_.getNumSubscribers()) {
    submap_vis_.publishPoseHistory(*submap_collection_ptr_,
                                   frame_names_.output_odom_frame,
                                   pose_history_pub_);
  }

  // Publish the loop closure edges
  loop_closure_edge_server_.publishLoopClosureEdges(
      pose_graph_interface_, *submap_collection_ptr_, current_timestamp);
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
                               "submap_" + std::to_string(submap_ptr->getID()),
                               false, current_timestamp);
  }
  if (!submap_collection_ptr_->empty()) {
    const SubmapID first_submap_id = submap_collection_ptr_->getFirstSubmapId();
    const VoxgraphSubmap& first_submap =
        submap_collection_ptr_->getSubmap(first_submap_id);
    if (!first_submap.getPoseHistory().empty()) {
      const Transformation T_odom__initial_pose =
          first_submap.getPose() *
          first_submap.getPoseHistory().begin()->second;
      TfHelper::publishTransform(T_odom__initial_pose,
                                 frame_names_.output_odom_frame, "initial_pose",
                                 false, current_timestamp);
    }
  }
}
}  // namespace voxgraph
