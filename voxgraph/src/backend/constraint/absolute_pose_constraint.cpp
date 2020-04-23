#include "voxgraph/backend/constraint/absolute_pose_constraint.h"

#include "voxgraph/backend/constraint/cost_functions/relative_pose_cost_function.h"

namespace voxgraph {
void AbsolutePoseConstraint::addToProblem(const NodeCollection& node_collection,
                                          ceres::Problem* problem) {
  CHECK_NOTNULL(problem);

  ceres::LossFunction* loss_function = kNoRobustLossFunction;

  // Get pointers to both submap nodes
  ReferenceFrameNode::Ptr reference_frame_node_ptr =
      node_collection.getReferenceFrameNodePtrById(config_.reference_frame_id);
  SubmapNode::Ptr submap_node_ptr =
      node_collection.getSubmapNodePtrById(config_.submap_id);
  CHECK_NOTNULL(reference_frame_node_ptr);
  CHECK_NOTNULL(submap_node_ptr);

  // Add the submap parameters to the problem
  reference_frame_node_ptr->addToProblem(
      problem, node_collection.getLocalParameterization());
  submap_node_ptr->addToProblem(problem,
                                node_collection.getLocalParameterization());

  // Add the constraint to the optimization and keep track of it
  ceres::CostFunction* cost_function = RelativePoseCostFunction::Create(
      config_.T_ref_submap, sqrt_information_matrix_);
  residual_block_id_ = problem->AddResidualBlock(
      cost_function, loss_function,
      reference_frame_node_ptr->getPosePtr()->optimizationVectorData(),
      submap_node_ptr->getPosePtr()->optimizationVectorData());
}
}  // namespace voxgraph
