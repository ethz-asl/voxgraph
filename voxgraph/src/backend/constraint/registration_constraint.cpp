//
// Created by victor on 17.01.19.
//

#include "voxgraph/backend/constraint/registration_constraint.h"
#include "voxgraph/backend/constraint/cost_functions/submap_registration/explicit_implicit_registration_cost.h"
#include "voxgraph/backend/constraint/cost_functions/submap_registration/implicit_implicit_registration_cost.h"

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

  // Create submap alignment cost function
  ceres::CostFunction *cost_function;
  if (config_.registration.jacobian_evaluation_method ==
      RegistrationCost::JacobianEvaluationMethod::kNumeric) {
    cost_function = nullptr;
    LOG(FATAL) << "Numeric cost not yet implemented";
  } else {
    if (config_.registration.registration_method ==
        RegistrationCost::RegistrationMethod::kExplicitToImplicit) {
      cost_function = new ExplicitImplicitRegistrationCost(
          config_.first_submap_ptr, config_.second_submap_ptr,
          config_.registration);
    } else {
      cost_function = new ImplicitImplicitRegistrationCost(
          config_.first_submap_ptr, config_.second_submap_ptr,
          config_.registration);
    }
  }

  // Add the constraint to the optimization and keep track of it
  residual_block_id_ = problem->AddResidualBlock(
      cost_function, loss_function, first_submap_node_ptr->getPosePtr()->data(),
      second_submap_node_ptr->getPosePtr()->data());
}
}  // namespace voxgraph
