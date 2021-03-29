#include "voxgraph/frontend/pose_graph_interface/pose_graph_manager.h"

#include <utility>
#include <vector>

namespace voxgraph {
PoseGraphManager::PoseGraphManager(
    ros::NodeHandle node_handle,
    VoxgraphSubmapCollection::Ptr submap_collection_ptr,
    voxblox::MeshIntegratorConfig mesh_config,
    std::string visualizations_odom_frame, bool verbose)
    : verbose_(verbose),
      submap_collection_ptr_(std::move(submap_collection_ptr)),
      sliding_pose_graph_("sliding pose graph"),
      full_pose_graph_("full pose graph"),
      visualization_odom_frame_(std::move(visualizations_odom_frame)),
      submap_vis_(submap_collection_ptr_->getConfig(), mesh_config) {
  // Advertise the pose graph visuals publisher
  pose_graph_pub_ = node_handle.advertise<visualization_msgs::Marker>(
      "pose_graph", 100, true);
  submap_pub_ =
      node_handle.advertise<visualization_msgs::Marker>("submap_info", 1, true);

  // Configure the sliding pose graph optimization
  PoseGraph::Config sliding_pose_graph_config;
  sliding_pose_graph_config.num_threads = 1;
  sliding_pose_graph_config.max_solver_time_in_seconds = 4.0;
  sliding_pose_graph_config.solver_type =
      PoseGraph::Config::SolverType::kDenseSchur;
  sliding_pose_graph_.setConfig(sliding_pose_graph_config);

  // Configure the full pose graph optimization
  PoseGraph::Config full_pose_graph_config;
  full_pose_graph_config.num_threads = 1;
  full_pose_graph_config.max_num_iterations = 10;
  full_pose_graph_config.solver_type =
      PoseGraph::Config::SolverType::kSparseSchur;
  full_pose_graph_.setConfig(full_pose_graph_config);
}

void PoseGraphManager::addSubmap(SubmapID submap_id) {
  // Avoid race conditions with the full pose graph optimization thread
  std::lock_guard<std::mutex> pose_graph_merging_lock(
      pose_graph_merging_mutex_);

  // Configure the submap node and add it to the pose graph
  SubmapNode::Config node_config = node_templates_.submap;
  node_config.submap_id = submap_id;
  // Although the submap collection is in robocentric frame, the pose graph
  // optimization is run in an inertial frame to avoid moving all submaps to
  // follow the drift on the new submap
  if (submap_collection_ptr_->size() == 1u) {
    // Use the first submap as the inertial frame origin,
    // and fix its pose in the optimization
    node_config.set_constant = true;
  }
  const VoxgraphSubmap& submap = submap_collection_ptr_->getSubmap(submap_id);
  SubmapID previous_submap_id;
  if (submap_collection_ptr_->getPreviousSubmapId(submap.getRobotName(),
                                                  &previous_submap_id)) {
    // Get the transformation from the current to the previous submap
    const VoxgraphSubmap& previous_submap =
        submap_collection_ptr_->getSubmap(previous_submap_id);
    Transformation T_O_previous_submap = previous_submap.getInitialPose();
    Transformation T_O_current_submap = submap.getInitialPose();
    const Transformation T_previous_current_submap =
        T_O_previous_submap.inverse() * T_O_current_submap;

    // Transform the current submap pose into inertial frame
    Transformation T_I_previous_submap;
    CHECK(sliding_pose_graph_.getSubmapPose(previous_submap_id,
                                            &T_I_previous_submap));
    const Transformation T_I_current_submap =
        T_I_previous_submap * T_previous_current_submap;

    node_config.T_I_node_initial = T_I_current_submap;
    node_config.set_constant = false;

    // Lock the previous submap in the sliding window if appropriate
    constexpr bool kOnlyOptimizeNewestSubmapInSlidingPoseGraph = false;
    if (kOnlyOptimizeNewestSubmapInSlidingPoseGraph) {
      sliding_pose_graph_.setSubmapNodeConstant(previous_submap_id, true);
    }
  } else {
    node_config.T_I_node_initial = submap.getInitialPose();
  }
  sliding_pose_graph_.addSubmapNode(node_config);

  ROS_INFO_STREAM_COND(verbose_,
                       "Added node to graph for submap: " << submap_id);
}

void PoseGraphManager::addOdometryMeasurement(const SubmapID& first_submap_id,
                                              const SubmapID& second_submap_id,
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
  sliding_pose_graph_.addRelativePoseConstraint(constraint_config);
}

void PoseGraphManager::addLoopClosureMeasurement(
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
  sliding_pose_graph_.addRelativePoseConstraint(constraint_config);
}

void PoseGraphManager::addHeightMeasurement(const SubmapID& submap_id,
                                            const double& height) {
  // Configure the constraint
  AbsolutePoseConstraint::Config constraint_config =
      measurement_templates_.height;
  constraint_config.submap_id = submap_id;
  constraint_config.T_ref_submap.getPosition().z() = height;

  // Add the odom reference frame to the pose graph if it isn't already there
  addReferenceFrameIfMissing(constraint_config.reference_frame_id);

  // Add the height measurement to the pose graph
  sliding_pose_graph_.addAbsolutePoseConstraint(constraint_config);
}

void PoseGraphManager::updateSlidingPoseGraphRegistrationConstraints() {
  if (registration_constraints_enabled_) {
    updateRegistrationConstraintsForOverlappingSubmapPairs(
        &sliding_pose_graph_);
  }
}

void PoseGraphManager::updateFullPoseGraphRegistrationConstraints() {
  if (registration_constraints_enabled_) {
    updateRegistrationConstraintsForOverlappingSubmapPairs(&full_pose_graph_);
  }
}

void PoseGraphManager::absorbSlidingWindowIntoFullPoseGraph() {
  // Copy the new nodes and carry over all poses
  for (const auto& submap_node_kv : sliding_pose_graph_.getSubmapNodes()) {
    // Add the node if needed
    if (!full_pose_graph_.hasSubmapNode(submap_node_kv.first)) {
      full_pose_graph_.addSubmapNode(submap_node_kv.second->getConfig());
    }
    // Set the submap pose to constant in the sliding pose graph,
    // and non-constant in the full pose graph unless it's the first submap
    sliding_pose_graph_.setSubmapNodeConstant(submap_node_kv.first, true);
    full_pose_graph_.setSubmapNodeConstant(submap_node_kv.first,
                                           submap_node_kv.first == 0u);

    // Carry over the pose
    full_pose_graph_.setSubmapPose(submap_node_kv.first,
                                   submap_node_kv.second->getPose());
  }
  for (const auto& reference_frame_node_kv :
       sliding_pose_graph_.getReferenceFrameNodes()) {
    // Add the node if needed
    if (!full_pose_graph_.hasReferenceFrameNode(
            reference_frame_node_kv.first)) {
      full_pose_graph_.addReferenceFrameNode(
          reference_frame_node_kv.second->getConfig());
    }
    // Carry over the pose
    full_pose_graph_.setReferenceFramePose(
        reference_frame_node_kv.first,
        reference_frame_node_kv.second->getPose());
  }

  // Move the new constraints
  // NOTE: We skip moving registration constraints since these are
  //       automatically redetected
  for (const AbsolutePoseConstraint& absolute_pose_constraint :
       sliding_pose_graph_.getAbsolutePoseConstraints()) {
    full_pose_graph_.addAbsolutePoseConstraint(
        absolute_pose_constraint.getConfig());
  }
  sliding_pose_graph_.resetAbsolutePoseConstraints();
  for (const RelativePoseConstraint& relative_pose_constraint :
       sliding_pose_graph_.getRelativePoseConstraints()) {
    full_pose_graph_.addRelativePoseConstraint(
        relative_pose_constraint.getConfig());
  }
  sliding_pose_graph_.resetRelativePoseConstraints();
  sliding_pose_graph_.resetRegistrationConstraints();
}

void PoseGraphManager::optimizeSlidingPoseGraph() {
  // Avoid race conditions and pose inconsistencies
  // with the full pose graph optimization thread
  std::lock_guard<std::mutex> pose_graph_merging_lock(
      pose_graph_merging_mutex_);

  optimizeSlidingPoseGraphImpl();

  // Publish debug visuals
  if (pose_graph_pub_.getNumSubscribers() > 0) {
    SubmapID last_submap_id;
    if (submap_collection_ptr_->getLastSubmapId(&last_submap_id)) {
      Transformation T_I_last_submap, T_O_last_submap;
      if (sliding_pose_graph_.getSubmapPose(last_submap_id, &T_I_last_submap) &&
          submap_collection_ptr_->getSubmapPose(last_submap_id,
                                                &T_O_last_submap)) {
        const Transformation T_O_I =
            T_O_last_submap * T_I_last_submap.inverse();
        pose_graph_vis_.publishPoseGraph(sliding_pose_graph_, T_O_I,
                                         visualization_odom_frame_,
                                         "sliding_pose_graph", pose_graph_pub_);
      } else {
        ROS_WARN_STREAM(
            "Could not get the optimized or original pose of "
            "submap ID "
            << last_submap_id << " for pose graph visualization");
      }
    }
  }
}

void PoseGraphManager::optimizeFullPoseGraph() {
  // Optimize the full pose graph
  full_pose_graph_.optimize();

  // Publish debug visuals
  if (pose_graph_pub_.getNumSubscribers() > 0) {
    SubmapID last_submap_id;
    if (submap_collection_ptr_->getLastSubmapId(&last_submap_id)) {
      Transformation T_I_last_submap, T_O_last_submap;
      if (sliding_pose_graph_.getSubmapPose(last_submap_id, &T_I_last_submap) &&
          submap_collection_ptr_->getSubmapPose(last_submap_id,
                                                &T_O_last_submap)) {
        const Transformation T_O_I =
            T_O_last_submap * T_I_last_submap.inverse();
        pose_graph_vis_.publishPoseGraph(full_pose_graph_, T_O_I,
                                         visualization_odom_frame_,
                                         "full_pose_graph", pose_graph_pub_);
      } else {
        ROS_WARN_STREAM(
            "Could not get the optimized or original pose of "
            "submap ID "
            << last_submap_id << " for pose graph visualization");
      }
    }
  }

  // Update the sliding pose graph and reoptimize the submaps that were added
  // while the full optimization was running
  {
    // Avoid race conditions with the sliding pose graph optimization thread
    std::lock_guard<std::mutex> pose_graph_merging_lock(
        pose_graph_merging_mutex_);

    // Compute a rigid alignment from the new submap nodes
    // in the sliding pose graph to the full pose graph
    bool found_valid_rigid_alignment = false;
    Transformation averaged_T_sliding_full;
    int num_old_overlapping_submaps = 0;
    updateSlidingPoseGraphRegistrationConstraints();
    for (const auto& registration_constraint :
         sliding_pose_graph_.getRegistrationConstraints()) {
      if (full_pose_graph_.hasSubmapNode(
              registration_constraint.getConfig().first_submap_id)) {
        const SubmapID old_overlapping_submap_id =
            registration_constraint.getConfig().first_submap_id;
        ++num_old_overlapping_submaps;
        found_valid_rigid_alignment = true;

        Transformation T_I_submap_full, T_I_submap_sliding;
        if (full_pose_graph_.getSubmapPose(old_overlapping_submap_id,
                                           &T_I_submap_full) &&
            sliding_pose_graph_.getSubmapPose(old_overlapping_submap_id,
                                              &T_I_submap_sliding)) {
          const Transformation T_sliding_full =
              T_I_submap_sliding.inverse() * T_I_submap_full;
          const double lambda = 1.0 / num_old_overlapping_submaps;
          averaged_T_sliding_full = kindr::minimal::interpolateComponentwise(
              averaged_T_sliding_full, T_sliding_full, lambda);
        } else {
          found_valid_rigid_alignment = false;
          ROS_WARN_STREAM(
              "Could not get the optimized full or sliding pose for "
              "submap ID: "
              << old_overlapping_submap_id);
          break;
        }
      }
    }
    if (found_valid_rigid_alignment) {
      ROS_INFO_STREAM(
          "Found valid rigid alignment between new sliding "
          "pose graph nodes to full pose graph, based on "
          << num_old_overlapping_submaps << " overlapping submaps:\n"
          << averaged_T_sliding_full);
    }

    // Update the sliding pose graph's poses
    for (const auto& submap_node_kv : sliding_pose_graph_.getSubmapNodes()) {
      const auto full_pose_graph_submap_node_it =
          full_pose_graph_.getSubmapNodes().find(submap_node_kv.first);
      if (full_pose_graph_submap_node_it !=
          full_pose_graph_.getSubmapNodes().end()) {
        // Carry over the pose
        const Transformation T_I_submap_full =
            full_pose_graph_submap_node_it->second->getPose();
        sliding_pose_graph_.setSubmapPose(submap_node_kv.first,
                                          T_I_submap_full);
      } else {
        if (found_valid_rigid_alignment) {
          // Compute an initial guess for the pose based on rigid alignment
          const Transformation T_I_submap_sliding =
              submap_node_kv.second->getPose();
          sliding_pose_graph_.setSubmapPose(
              submap_node_kv.first,
              T_I_submap_sliding * averaged_T_sliding_full);
          // TODO(victorr): Check if this really does the right thing
        } else {
          ROS_WARN_STREAM(
              "Not computing an initial guess for pose of recently added "
              "submap "
              << submap_node_kv.first
              << " since no usable rigid alignment was found between the "
                 "sliding and full pose graph.");
        }
      }
    }
    for (const auto& reference_frame_node_kv :
         full_pose_graph_.getReferenceFrameNodes()) {
      // Carry over the pose if the node also exists in the sliding pose graph
      if (sliding_pose_graph_.hasReferenceFrameNode(
              reference_frame_node_kv.first)) {
        sliding_pose_graph_.setReferenceFramePose(
            reference_frame_node_kv.first,
            reference_frame_node_kv.second->getPose());
      }
    }

    // Reoptimize the sliding pose graph, if it has any new (non-constant) nodes
    // NOTE: Optimizing the sliding pose graph will also update
    //       the submap_collection poses
    const std::map<SubmapID, bool> sliding_submap_node_constness_map =
        sliding_pose_graph_.getSubmapNodeConstness();
    if (!sliding_submap_node_constness_map.empty() &&
        std::any_of(
            sliding_submap_node_constness_map.begin(),
            sliding_submap_node_constness_map.end(),
            [](const std::pair<SubmapID, bool>& kv) { return !kv.second; })) {
      updateSlidingPoseGraphRegistrationConstraints();
      optimizeSlidingPoseGraphImpl();
    }
  }
}

// bool PoseGraphManager::getEdgeCovarianceMap(
//    PoseGraph::EdgeCovarianceMap* edge_covariance_map_ptr) const {
//  CHECK_NOTNULL(edge_covariance_map_ptr);
//
//  // Request covariance estimates for all overlapping submap pairs
//  for (const SubmapIdPair& overlapping_submap_pair : overlapping_submap_list_)
//  {
//    edge_covariance_map_ptr->emplace(overlapping_submap_pair,
//                                     PoseGraph::EdgeCovarianceMatrix::Zero());
//  }
//
//  return sliding_pose_graph_.getEdgeCovarianceMap(edge_covariance_map_ptr);
//}

void PoseGraphManager::updateSubmapCollectionPosesBasedOnSlidingPoseGraph() {
  // Convert the poses from inertial to robocentric frame
  SubmapID last_submap_id =
      sliding_pose_graph_.getSubmapPoses().rbegin()->first;
  Transformation T_I_last_submap, T_O_last_submap;
  if (sliding_pose_graph_.getSubmapPose(last_submap_id, &T_I_last_submap) &&
      submap_collection_ptr_->getSubmapPose(last_submap_id, &T_O_last_submap)) {
    const Transformation T_O_I = T_O_last_submap * T_I_last_submap.inverse();
    for (const auto& submap_pose_kv : sliding_pose_graph_.getSubmapPoses()) {
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

void PoseGraphManager::updateRegistrationConstraintsForOverlappingSubmapPairs(
    PoseGraph* pose_graph_ptr) {
  CHECK_NOTNULL(pose_graph_ptr);

  // Remove the old registration constraints
  pose_graph_ptr->resetRegistrationConstraints();

  // Get all submap nodes, including info on whether their pose is set constant
  PoseGraph::SubmapNodeConstnessMap submap_constness_map =
      pose_graph_ptr->getSubmapNodeConstness();

  // For each submap in the collection, check which submaps it overlaps with
  for (auto first_submap_constness_it = submap_constness_map.cbegin();
       first_submap_constness_it != submap_constness_map.cend();
       ++first_submap_constness_it) {
    // Get a pointer to the first submap
    const cblox::SubmapID first_submap_id = first_submap_constness_it->first;
    const VoxgraphSubmap& first_submap =
        submap_collection_ptr_->getSubmap(first_submap_id);

    // Test the current submaps against all subsequent submaps
    // NOTE: Testing against the subsequent submaps only is sufficient to test
    //       all pairs in the submap collection, since the overlap test is
    //       symmetric (i.e. pair A<->B and B<->A are equivalent)
    for (auto second_submap_constness_it = std::next(first_submap_constness_it);
         second_submap_constness_it != submap_constness_map.cend();
         ++second_submap_constness_it) {
      // Get the second submap
      const cblox::SubmapID second_submap_id =
          second_submap_constness_it->first;
      const VoxgraphSubmap& second_submap =
          submap_collection_ptr_->getSubmap(second_submap_id);

      // Only consider constraining pairs with at least one non-constant node
      if (!first_submap_constness_it->second ||
          !second_submap_constness_it->second) {
        // Check whether the first and second submap overlap
        if (first_submap.overlapsWith(second_submap)) {
          // Add the registration constraint to the pose graph
          addRegistrationConstraintForSubmapPair(
              first_submap_id, second_submap_id, pose_graph_ptr);
        }
      }
    }
  }
}

void PoseGraphManager::addRegistrationConstraintForSubmapPair(
    const SubmapID first_submap_id, const SubmapID second_submap_id,
    PoseGraph* pose_graph_ptr) {
  // Configure the registration constraint
  RegistrationConstraint::Config constraint_config =
      measurement_templates_.registration;
  constraint_config.first_submap_id = first_submap_id;
  constraint_config.second_submap_id = second_submap_id;

  // Add pointers to both submaps
  constraint_config.first_submap_ptr =
      submap_collection_ptr_->getSubmapConstPtr(first_submap_id);
  constraint_config.second_submap_ptr =
      submap_collection_ptr_->getSubmapConstPtr(second_submap_id);
  CHECK_NOTNULL(constraint_config.first_submap_ptr);
  CHECK_NOTNULL(constraint_config.second_submap_ptr);

  // Mirror the constraint to account for the
  // assymetry of implicit registration
  RegistrationConstraint::Config mirrored_config = constraint_config;
  mirrored_config.first_submap_id = constraint_config.second_submap_id;
  mirrored_config.first_submap_ptr = constraint_config.second_submap_ptr;
  mirrored_config.second_submap_id = constraint_config.first_submap_id;
  mirrored_config.second_submap_ptr = constraint_config.first_submap_ptr;

  // Add the constraint to the pose graph
  pose_graph_ptr->addRegistrationConstraint(constraint_config);
  pose_graph_ptr->addRegistrationConstraint(mirrored_config);
}

void PoseGraphManager::addReferenceFrameIfMissing(
    ReferenceFrameNode::FrameId frame_id) {
  if (!sliding_pose_graph_.hasReferenceFrameNode(frame_id)) {
    sliding_pose_graph_.addReferenceFrameNode(
        node_templates_.getReferenceFrameConfigById(frame_id));
  }
}

void PoseGraphManager::optimizeSlidingPoseGraphImpl() {
  // Optimize a reduced pose graph to align only the last submap
  sliding_pose_graph_.optimize();

  // Update the submap poses
  updateSubmapCollectionPosesBasedOnSlidingPoseGraph();
}
}  // namespace voxgraph
