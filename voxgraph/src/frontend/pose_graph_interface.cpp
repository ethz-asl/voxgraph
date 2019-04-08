//
// Created by victor on 09.04.19.
//

#include "voxgraph/frontend/pose_graph_interface.h"
#include <vector>

namespace voxgraph {
void PoseGraphInterface::addSubmap(SubmapID submap_id) {
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
