//
// Created by victor on 09.04.19.
//

#include "voxgraph/frontend/pose_graph_interface.h"
#include <vector>

namespace voxgraph {
void PoseGraphInterface::addSubmap(SubmapID submap_id, bool add_easy_odometry) {
  // Generate the submap's ESDF
  submap_collection_ptr_->generateEsdfById(submap_id);
  // TODO(victorr): Parametrize this s.t. it doesn't happen
  //                when using TSDF registration

  // Configure the submap node and add it to the pose graph
  SubmapNode::Config submap_node_config;
  submap_node_config.submap_id = submap_id;
  CHECK(submap_collection_ptr_->getSubMapPose(
      submap_id, &submap_node_config.initial_submap_pose));
  if (submap_id == 0) {
    std::cout << "Setting pose of submap 0 to constant" << std::endl;
    submap_node_config.set_constant = true;
  } else {
    submap_node_config.set_constant = false;
  }
  pose_graph_.addSubmapNode(submap_node_config);

  // Easy way to add an odometry constraint between the previous and new submap
  // NOTE: This method assumes that the current submap pose purely comes from
  //       odometry and has not yet been corrected through other means
  if (add_easy_odometry && submap_collection_ptr_->size() >= 2) {
    SubmapID previous_submap_id = submap_collection_ptr_->getPreviousSubmapId();

    // Configure the odometry constraint
    OdometryConstraint::Config odom_constraint_config;
    odom_constraint_config.origin_submap_id = previous_submap_id;
    odom_constraint_config.destination_submap_id = submap_id;
    // TODO(victorr): Properly tune the odometry information matrix
    odom_constraint_config.information_matrix =
        1 * odom_constraint_config.information_matrix.setIdentity();
    odom_constraint_config.information_matrix(3, 3) = 1e3;

    // Set the relative transformation
    Transformation T_world__previous_submap;
    Transformation T_world__current_submap;
    CHECK(submap_collection_ptr_->getSubMapPose(previous_submap_id,
                                                &T_world__previous_submap));
    CHECK(submap_collection_ptr_->getSubMapPose(submap_id,
                                                &T_world__current_submap));
    odom_constraint_config.T_origin_destination =
        T_world__previous_submap.inverse() * T_world__current_submap;

    // Add the odometry constraint to the pose graph
    if (verbose_) {
      std::cout << "Adding odom constraint\n"
                << "From: " << odom_constraint_config.origin_submap_id << "\n"
                << "To: " << odom_constraint_config.destination_submap_id
                << "\n"
                << "T_w_s1:\n"
                << T_world__previous_submap << "\n"
                << "yaw_w_s1:" << T_world__previous_submap.log()[5] << "\n"
                << "T_w_s2:\n"
                << T_world__current_submap << "\n"
                << "yaw_w_s2:" << T_world__current_submap.log()[5] << "\n"
                << "T_s1_s2:\n"
                << odom_constraint_config.T_origin_destination << "\n"
                << "yaw_s1_s2: "
                << odom_constraint_config.T_origin_destination.log()[5] << "\n"
                << "Information matrix\n"
                << odom_constraint_config.information_matrix << std::endl;
    }

    pose_graph_.addOdometryConstraint(odom_constraint_config);
  }
}

void PoseGraphInterface::optimize() {
  updateRegistrationConstraints();
  pose_graph_.optimize();
}

void PoseGraphInterface::updateSubmapCollectionPoses() {
  for (const auto& submap_pose_kv : pose_graph_.getSubmapPoses()) {
    submap_collection_ptr_->setSubMapPose(submap_pose_kv.first,
                                          submap_pose_kv.second);
  }
}

void PoseGraphInterface::updateRegistrationConstraints() {
  // Constrain all overlapping submaps to each other
  pose_graph_.resetRegistrationConstraints();
  std::vector<cblox::SubmapID> submap_ids = submap_collection_ptr_->getIDs();
  for (unsigned int i = 0; i < submap_ids.size(); i++) {
    // Get a pointer to the first submap
    cblox::SubmapID first_submap_id = submap_ids[i];
    const VoxgraphSubmap& first_submap =
        submap_collection_ptr_->getSubMap(first_submap_id);

    for (unsigned int j = i + 1; j < submap_ids.size(); j++) {
      // Get the second submap
      cblox::SubmapID second_submap_id = submap_ids[j];
      const VoxgraphSubmap& second_submap =
          submap_collection_ptr_->getSubMap(second_submap_id);

      // Check whether the first and second submap overlap
      if (first_submap.overlapsWith(second_submap)) {
        // Configure the registration constraint
        RegistrationConstraint::Config registration_constraint_config;
        registration_constraint_config.first_submap_id = first_submap_id;
        registration_constraint_config.second_submap_id = second_submap_id;
        registration_constraint_config.information_matrix.setIdentity();

        // Add pointers to both submaps
        registration_constraint_config.first_submap_ptr =
            submap_collection_ptr_->getSubMapConstPtrById(first_submap_id);
        registration_constraint_config.second_submap_ptr =
            submap_collection_ptr_->getSubMapConstPtrById(second_submap_id);

        CHECK_NOTNULL(registration_constraint_config.first_submap_ptr);
        CHECK_NOTNULL(registration_constraint_config.second_submap_ptr);

        // Add the constraint to the pose graph
        pose_graph_.addRegistrationConstraint(registration_constraint_config);
      }
    }
  }
}
}  // namespace voxgraph
