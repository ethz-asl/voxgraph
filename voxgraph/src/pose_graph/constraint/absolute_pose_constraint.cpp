//
// Created by victor on 03.04.19.
//

#include "voxgraph/pose_graph/constraint/absolute_pose_constraint.h"
#include "voxgraph/pose_graph/constraint/cost_functions/relative_pose_cost_function.h"

namespace voxgraph {
void AbsolutePoseConstraint::addToProblem(const NodeCollection &node_collection,
                                          ceres::Problem *problem) {
  CHECK_NOTNULL(problem);

  ceres::LossFunction *loss_function = nullptr;

  // Get pointers to both submap nodes
  SubmapNode::Ptr reference_frame_node_ptr =
      node_collection.getNodePtrBySubmapId(config_.reference_frame_id);
  SubmapNode::Ptr submap_node_ptr =
      node_collection.getNodePtrBySubmapId(config_.submap_id);
  CHECK_NOTNULL(reference_frame_node_ptr);
  CHECK_NOTNULL(submap_node_ptr);

  // Add the submap parameters to the problem
  reference_frame_node_ptr->addToProblem(problem);
  submap_node_ptr->addToProblem(problem);

  // Add the cost function to the problem
  ceres::CostFunction *cost_function = RelativePoseCostFunction::Create(
      config_.T_ref_submap, sqrt_information_matrix_);
  problem->AddResidualBlock(cost_function, loss_function,
                            reference_frame_node_ptr->getPosePtr()->data(),
                            submap_node_ptr->getPosePtr()->data());
}
}  // namespace voxgraph
