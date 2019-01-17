//
// Created by victor on 17.01.19.
//

#include "voxgraph/pose_graph/registration_constraint.h"

namespace voxgraph {
void RegistrationConstraint::addToProblem(const Node::NodeMap &node_map,
                                          ceres::Problem *problem) {
  CHECK_NOTNULL(problem);

  ceres::LossFunction *loss_function = nullptr;

  // Get pointers to the poses of both endpoints
  Node::Pose *first_node_pose, *second_node_pose;
  {
    auto node_map_it = node_map.find(endpoints_.first_node_id);
    // First node
    CHECK(node_map_it != node_map.end()) << "Graph contains no node for submap "
                                         << config_.first_submap_id;
    first_node_pose = node_map_it->second->getPosePtr();
    // Second node
    node_map_it = node_map.find(endpoints_.second_node_id);
    CHECK(node_map_it != node_map.end()) << "Graph contains no node for submap "
                                         << config_.second_submap_id;
    second_node_pose = node_map_it->second->getPosePtr();
  }
  CHECK_NOTNULL(first_node_pose);
  CHECK_NOTNULL(second_node_pose);
  // Add the submap parameters tot the problem
  problem->AddParameterBlock(first_node_pose->data(), first_node_pose->size());
  problem->AddParameterBlock(second_node_pose->data(),
                             second_node_pose->size());
  // TODO(victorr): Implement the option to add constant poses

  // TODO(victorr): Handle cost_options properly
  SubmapRegisterer::Options::CostFunction cost_options;

  // Create submap alignment cost function
  ceres::CostFunction *cost_function;
  if (cost_options.cost_function_type ==
      SubmapRegisterer::Options::CostFunction::Type::kNumeric) {
    cost_function = nullptr;
    LOG(FATAL) << "Numeric cost not yet implemented";
  } else {
    cost_function = new RegistrationCostFunction(
        first_submap_ptr_, second_submap_ptr_, cost_options);
  }

  // Add the constraint to the optimization
  problem->AddResidualBlock(cost_function, loss_function,
                            first_node_pose->data(), second_node_pose->data());
}
}  // namespace voxgraph
