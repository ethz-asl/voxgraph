//
// Created by victor on 03.04.19.
//

#include "voxgraph/backend/constraint/loop_closure_constraint.h"
#include "voxgraph/backend/constraint/cost_functions/relative_pose_cost_function.h"

namespace voxgraph {
void LoopClosureConstraint::addToProblem(const NodeCollection &node_collection,
                                         ceres::Problem *problem) {
  CHECK_NOTNULL(problem);

  ceres::LossFunction *loss_function = nullptr;

  // Get pointers to both submap nodes
  SubmapNode::Ptr origin_submap_node_ptr =
      node_collection.getNodePtrBySubmapId(config_.origin_submap_id);
  SubmapNode::Ptr destination_submap_node_ptr =
      node_collection.getNodePtrBySubmapId(config_.destination_submap_id);
  CHECK_NOTNULL(origin_submap_node_ptr);
  CHECK_NOTNULL(destination_submap_node_ptr);

  // Add the submap parameters to the problem
  origin_submap_node_ptr->addToProblem(problem);
  destination_submap_node_ptr->addToProblem(problem);

  // Add the cost function to the problem
  ceres::CostFunction *cost_function = RelativePoseCostFunction::Create(
      config_.T_origin_destination, sqrt_information_matrix_);
  problem->AddResidualBlock(cost_function, loss_function,
                            origin_submap_node_ptr->getPosePtr()->data(),
                            destination_submap_node_ptr->getPosePtr()->data());
}
}  // namespace voxgraph
