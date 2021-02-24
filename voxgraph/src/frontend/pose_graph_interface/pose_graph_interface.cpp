#include "voxgraph/frontend/pose_graph_interface/pose_graph_interface.h"

#include <utility>
#include <vector>

namespace voxgraph {
PoseGraphInterface::PoseGraphInterface(
    ros::NodeHandle node_handle,
    VoxgraphSubmapCollection::Ptr submap_collection_ptr,
    voxblox::MeshIntegratorConfig mesh_config,
    const std::string& visualizations_odom_frame, bool verbose)
    : verbose_(verbose),
      submap_collection_ptr_(std::move(submap_collection_ptr)),
      visualization_odom_frame_(visualizations_odom_frame),
      submap_vis_(submap_collection_ptr_->getConfig(), mesh_config),
      new_loop_closures_added_since_last_optimization_(false) {
  // Advertise the pose graph visuals publisher
  pose_graph_pub_ = node_handle.advertise<visualization_msgs::Marker>(
      "pose_graph", 100, true);
  submap_pub_ =
      node_handle.advertise<visualization_msgs::Marker>("submap_info", 1, true);
}

void PoseGraphInterface::addSubmap(SubmapID submap_id) {
  // Configure the submap node and add it to the pose graph
  SubmapNode::Config node_config = node_templates_.submap;
  node_config.submap_id = submap_id;
  // Although the submap collection is in robocentric frame, the pose graph
  // optimization is run in an inertial frame to avoid moving all submaps to
  // follow the drift on the new submap
  if (submap_collection_ptr_->size() == 1u) {
    // Use the first submap as the inertial frame origin,
    // and fix its pose in the optimization
    node_config.T_I_node_initial = Transformation();
    node_config.set_constant = true;
  } else {
    // Get the transformation from the current to the previous submap
    SubmapID previous_submap_id = submap_collection_ptr_->getPreviousSubmapId();
    Transformation T_O_previous_submap;
    CHECK(submap_collection_ptr_->getSubmapPose(previous_submap_id,
                                                &T_O_previous_submap));
    Transformation T_O_current_submap;
    CHECK(
        submap_collection_ptr_->getSubmapPose(submap_id, &T_O_current_submap));
    const Transformation T_previous_current_submap =
        T_O_previous_submap.inverse() * T_O_current_submap;

    // Transform the current submap pose into inertial frame
    Transformation T_I_previous_submap;
    CHECK(pose_graph_.getSubmapPose(previous_submap_id, &T_I_previous_submap));
    const Transformation T_I_current_submap =
        T_I_previous_submap * T_previous_current_submap;

    node_config.T_I_node_initial = T_I_current_submap;
    node_config.set_constant = false;
  }
  pose_graph_.addSubmapNode(node_config);

  ROS_INFO_STREAM_COND(verbose_,
                       "Added node to graph for submap: " << submap_id);
}

void PoseGraphInterface::addOdometryMeasurement(
    const SubmapID& first_submap_id, const SubmapID& second_submap_id,
    const Transformation& T_S1_S2) {
  // Configure the odometry constraint
  RelativePoseConstraint::Config constraint_config =
      measurement_templates_.odometry;
  constraint_config.origin_submap_id = first_submap_id;
  constraint_config.destination_submap_id = second_submap_id;
  constraint_config.T_origin_destination = T_S1_S2;

  // Add the constraint to the pose graph
  if (verbose_) {
    std::cout << "Adding odom constraint\n"
              << "From: " << constraint_config.origin_submap_id << "\n"
              << "To: " << constraint_config.destination_submap_id << "\n"
              << "Submap currently being built in submap collection: "
              << submap_collection_ptr_->getActiveSubmapID() << "\n"
              << "t_s1_s2:\n"
              << constraint_config.T_origin_destination.getPosition() << "\n"
              << "yaw_s1_s2: "
              << constraint_config.T_origin_destination.log()[5] << "\n"
              << "Information matrix\n"
              << constraint_config.information_matrix << std::endl;
  }
  pose_graph_.addRelativePoseConstraint(constraint_config);
}

void PoseGraphInterface::addLoopClosureMeasurement(
    const SubmapID& from_submap, const SubmapID& to_submap,
    const Transformation& transform) {
  // Configure the loop closure constraint
  RelativePoseConstraint::Config constraint_config =
      measurement_templates_.loop_closure;
  constraint_config.origin_submap_id = from_submap;
  constraint_config.destination_submap_id = to_submap;
  constraint_config.T_origin_destination = transform;

  // Add the constraint to the pose graph
  ROS_INFO_STREAM_COND(verbose_,
                       "Adding loop closure as relative pose constraint "
                       "from submap "
                           << constraint_config.origin_submap_id << " to "
                           << constraint_config.destination_submap_id
                           << " with transform\n"
                           << constraint_config.T_origin_destination
                           << "\nand information matrix\n"
                           << constraint_config.information_matrix);
  pose_graph_.addRelativePoseConstraint(constraint_config);

  // Indicate that a new loop closure constraint has been added
  new_loop_closures_added_since_last_optimization_ = true;
}

void PoseGraphInterface::addHeightMeasurement(const SubmapID& submap_id,
                                              const double& height) {
  // Configure the constraint
  AbsolutePoseConstraint::Config constraint_config =
      measurement_templates_.height;
  constraint_config.submap_id = submap_id;
  constraint_config.T_ref_submap.getPosition().z() = height;

  // Add the odom reference frame to the pose graph if it isn't already there
  addReferenceFrameIfMissing(constraint_config.reference_frame_id);

  // Add the height measurement to the pose graph
  pose_graph_.addAbsolutePoseConstraint(constraint_config);
}

void PoseGraphInterface::updateOverlappingSubmapList() {
  // Clear the old overlapping submap list
  overlapping_submap_list_.clear();

  // For each submap in the collection, check which submaps it overlaps with
  std::vector<cblox::SubmapID> submap_ids = submap_collection_ptr_->getIDs();
  for (size_t i = 0; i < submap_ids.size(); i++) {
    // Get a pointer to the first submap
    cblox::SubmapID first_submap_id = submap_ids[i];
    const VoxgraphSubmap& first_submap =
        submap_collection_ptr_->getSubmap(first_submap_id);

    // Publish debug visuals
    // TODO(victorr): Move this to a better place (e.g. visualization class)
    if (submap_pub_.getNumSubscribers() > 0) {
      submap_vis_.publishBox(
          first_submap.getOdomFrameSurfaceAabb().getCornerCoordinates(),
          voxblox::Color::Blue(), visualization_odom_frame_,
          "surface_abb" + std::to_string(first_submap_id), submap_pub_);
    }

    // Test the current submaps against all subsequent submaps
    // NOTE: Testing against the subsequent submaps only is sufficient to test
    //       all pairs in the submap collection, since the overlap test is
    //       symmetric (i.e. pair A<->B and B<->A are equivalent)
    for (size_t j = i + 1; j < submap_ids.size(); j++) {
      // Get the second submap
      cblox::SubmapID second_submap_id = submap_ids[j];
      const VoxgraphSubmap& second_submap =
          submap_collection_ptr_->getSubmap(second_submap_id);

      // Check whether the first and second submap overlap
      if (first_submap.overlapsWith(second_submap)) {
        overlapping_submap_list_.emplace_back(first_submap_id,
                                              second_submap_id);
      }
    }
  }
}

void PoseGraphInterface::updateRegistrationConstraints() {
  // Remove the previous iteration's registration constraints
  pose_graph_.resetRegistrationConstraints();

  // Redetect which submaps overlap
  updateOverlappingSubmapList();

  // Add the updated registration constraints
  for (const SubmapIdPair& submap_pair : overlapping_submap_list_) {
    // Configure the registration constraint
    RegistrationConstraint::Config constraint_config =
        measurement_templates_.registration;
    constraint_config.first_submap_id = submap_pair.first;
    constraint_config.second_submap_id = submap_pair.second;

    // Add pointers to both submaps
    constraint_config.first_submap_ptr =
        submap_collection_ptr_->getSubmapConstPtr(submap_pair.first);
    constraint_config.second_submap_ptr =
        submap_collection_ptr_->getSubmapConstPtr(submap_pair.second);
    CHECK_NOTNULL(constraint_config.first_submap_ptr);
    CHECK_NOTNULL(constraint_config.second_submap_ptr);

    // Add the constraint to the pose graph
    pose_graph_.addRegistrationConstraint(constraint_config);
  }
}

void PoseGraphInterface::optimize() {
  // If new loop closures were added since the last optimization run,
  // preoptimize the graph without considering the registration constraints
  // NOTE: This is done to reduce the effect of registration constraints
  //       that strongly stick to local minima
  if (new_loop_closures_added_since_last_optimization_) {
    // Optimize the graph excluding the registration constraints
    pose_graph_.optimize(true);

    // Indicate that the new loop closures have been taken care off
    new_loop_closures_added_since_last_optimization_ = false;
  }

  // Optimize the pose graph with all constraints enabled
  pose_graph_.optimize();

  // Publish debug visuals
  if (pose_graph_pub_.getNumSubscribers() > 0) {
    SubmapID last_submap_id = submap_collection_ptr_->getLastSubmapId();
    Transformation T_I_last_submap, T_O_last_submap;
    if (pose_graph_.getSubmapPose(last_submap_id, &T_I_last_submap) &&
        submap_collection_ptr_->getSubmapPose(last_submap_id,
                                              &T_O_last_submap)) {
      const Transformation T_O_I = T_O_last_submap * T_I_last_submap.inverse();
      pose_graph_vis_.publishPoseGraph(pose_graph_, T_O_I,
                                       visualization_odom_frame_, "optimized",
                                       pose_graph_pub_);
    } else {
      ROS_WARN_STREAM(
          "Could not get the optimized or original pose of "
          "submap ID "
          << last_submap_id << " for pose graph visualization");
    }
  }
}

void PoseGraphInterface::updateSubmapCollectionPoses() {
  SubmapID last_submap_id = submap_collection_ptr_->getLastSubmapId();
  Transformation T_I_last_submap, T_O_last_submap;
  if (pose_graph_.getSubmapPose(last_submap_id, &T_I_last_submap) &&
      submap_collection_ptr_->getSubmapPose(last_submap_id, &T_O_last_submap)) {
    const Transformation T_O_I = T_O_last_submap * T_I_last_submap.inverse();
    for (const auto& submap_pose_kv : pose_graph_.getSubmapPoses()) {
      // Write back the updated pose,
      // after transforming them back into robocentric frame
      Transformation T_O_submap = T_O_I * submap_pose_kv.second;
      submap_collection_ptr_->setSubmapPose(submap_pose_kv.first, T_O_submap);
    }
  } else {
    ROS_WARN_STREAM(
        "Could not get the optimized or original pose for "
        "submap ID: "
        << last_submap_id);
  }
}

bool PoseGraphInterface::getEdgeCovarianceMap(
    PoseGraph::EdgeCovarianceMap* edge_covariance_map_ptr) const {
  CHECK_NOTNULL(edge_covariance_map_ptr);

  // Request covariance estimates for all overlapping submap pairs
  for (const SubmapIdPair& overlapping_submap_pair : overlapping_submap_list_) {
    edge_covariance_map_ptr->emplace(overlapping_submap_pair,
                                     PoseGraph::EdgeCovarianceMatrix::Zero());
  }

  return pose_graph_.getEdgeCovarianceMap(edge_covariance_map_ptr);
}

void PoseGraphInterface::addReferenceFrameIfMissing(
    ReferenceFrameNode::FrameId frame_id) {
  if (!pose_graph_.hasReferenceFrameNode(frame_id)) {
    pose_graph_.addReferenceFrameNode(
        node_templates_.getReferenceFrameConfigById(frame_id));
  }
}
}  // namespace voxgraph
