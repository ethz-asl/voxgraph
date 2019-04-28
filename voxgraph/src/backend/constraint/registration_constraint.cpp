//
// Created by victor on 17.01.19.
//

#include "voxgraph/backend/constraint/registration_constraint.h"

namespace voxgraph {
void RegistrationConstraint::addToProblem(const NodeCollection &node_collection,
                                          ceres::Problem *problem) {
  CHECK_NOTNULL(problem);

  ceres::LossFunction *loss_function = nullptr;

  // Get pointers to both submap nodes
  SubmapNode::Ptr first_submap_node_ptr =
      node_collection.getSubmapNodePtrById(config_.first_submap_id);
  SubmapNode::Ptr second_submap_node_ptr =
      node_collection.getSubmapNodePtrById(config_.second_submap_id);
  CHECK_NOTNULL(first_submap_node_ptr);
  CHECK_NOTNULL(second_submap_node_ptr);

  // Add the submap parameters to the problem
  first_submap_node_ptr->addToProblem(
      problem, node_collection.getLocalParameterization());
  second_submap_node_ptr->addToProblem(
      problem, node_collection.getLocalParameterization());

  // TODO(victorr): Load cost options from ROS params instead of using default
  SubmapRegisterer::Options::CostFunction cost_options;
  cost_options.registration_method =
      SubmapRegisterer::Options::CostFunction::kImplicitToImplicit;
  cost_options.jacobian_evaluation_method =
      SubmapRegisterer::Options::CostFunction::kAnalytic;
  cost_options.no_correspondence_cost = 0;
  cost_options.use_esdf_distance = true;
  cost_options.visualize_residuals = false;
  cost_options.visualize_gradients = false;
  cost_options.visualize_transforms_ = false;

  // Create submap alignment cost function
  ceres::CostFunction *cost_function;
  if (cost_options.jacobian_evaluation_method ==
      SubmapRegisterer::Options::CostFunction::JacobianEvaluationMethod::
          kNumeric) {
    cost_function = nullptr;
    LOG(FATAL) << "Numeric cost not yet implemented";
  } else {
    cost_function = new ImplicitImplicitRegistrationCostFn(
        config_.first_submap_ptr, config_.second_submap_ptr, cost_options);
  }

  // Add the constraint to the optimization and keep track of it
  residual_block_id_ = problem->AddResidualBlock(
      cost_function, loss_function, first_submap_node_ptr->getPosePtr()->data(),
      second_submap_node_ptr->getPosePtr()->data());
}
}  // namespace voxgraph
