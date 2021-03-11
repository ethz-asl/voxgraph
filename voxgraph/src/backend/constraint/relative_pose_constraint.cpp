#include "voxgraph/backend/constraint/relative_pose_constraint.h"

#include "voxgraph/backend/constraint/cost_functions/relative_pose_cost_function.h"

namespace voxgraph {
void RelativePoseConstraint::addToProblem(const NodeCollection& node_collection,
                                          ceres::Problem* problem,
                                          bool ignore_if_endpoints_constant) {
  CHECK_NOTNULL(problem);

  ceres::LossFunction* loss_function = kNoRobustLossFunction;

  // Get pointers to both submap nodes
  SubmapNode::Ptr origin_submap_node_ptr =
      node_collection.getSubmapNodePtrById(config_.origin_submap_id);
  SubmapNode::Ptr destination_submap_node_ptr =
      node_collection.getSubmapNodePtrById(config_.destination_submap_id);
  CHECK(origin_submap_node_ptr)
      << "Could not get pose graph submap node ptr for submap ID "
      << config_.origin_submap_id;
  CHECK(destination_submap_node_ptr)
      << "Could not get pose graph submap node ptr for submap ID "
      << config_.destination_submap_id;

  // Skip constraints that don't affect any non-constant pose graph nodes
  if (ignore_if_endpoints_constant && origin_submap_node_ptr->isConstant() &&
      destination_submap_node_ptr->isConstant()) {
    return;
  }

  // Add the submap parameters to the problem
  origin_submap_node_ptr->addToProblem(
      problem, node_collection.getLocalParameterization());
  destination_submap_node_ptr->addToProblem(
      problem, node_collection.getLocalParameterization());

  // Add the constraint to the optimization and keep track of it
  ceres::CostFunction* cost_function = RelativePoseCostFunction::Create(
      config_.T_origin_destination, sqrt_information_matrix_);
  residual_block_id_ = problem->AddResidualBlock(
      cost_function, loss_function,
      origin_submap_node_ptr->getPosePtr()->optimizationVectorData(),
      destination_submap_node_ptr->getPosePtr()->optimizationVectorData());
}
}  // namespace voxgraph
