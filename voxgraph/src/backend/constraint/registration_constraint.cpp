#include "voxgraph/backend/constraint/registration_constraint.h"

#include "voxgraph/backend/constraint/cost_functions/registration_cost_function.h"

namespace voxgraph {
RegistrationConstraint::RegistrationConstraint(
    Constraint::ConstraintId constraint_id,
    const RegistrationConstraint::Config& config)
    : Constraint(constraint_id, config), config_(config) {
  // Check whether both submap pointers have been provided
  CHECK_NOTNULL(config_.first_submap_ptr);
  CHECK_NOTNULL(config_.second_submap_ptr);

  // Registration constraints do not yet support non-identity information
  // matrices, therefore assert this
  CHECK(sqrt_information_matrix_.isIdentity())
      << "Registration constraint information matrices that differ "
         "from the identity matrix are not yet supported.";
}

void RegistrationConstraint::addToProblem(const NodeCollection& node_collection,
                                          ceres::Problem* problem,
                                          bool ignore_if_endpoints_constant) {
  CHECK_NOTNULL(problem);

  ceres::LossFunction* loss_function = kNoRobustLossFunction;

  // Get pointers to both submap nodes
  SubmapNode::Ptr first_submap_node_ptr =
      node_collection.getSubmapNodePtrById(config_.first_submap_id);
  SubmapNode::Ptr second_submap_node_ptr =
      node_collection.getSubmapNodePtrById(config_.second_submap_id);
  CHECK_NOTNULL(first_submap_node_ptr);
  CHECK_NOTNULL(second_submap_node_ptr);

  // Skip constraints that don't affect any non-constant pose graph nodes
  if (ignore_if_endpoints_constant && first_submap_node_ptr->isConstant() &&
      second_submap_node_ptr->isConstant()) {
    return;
  }

  // Add the submap parameters to the problem
  first_submap_node_ptr->addToProblem(
      problem, node_collection.getLocalParameterization());
  second_submap_node_ptr->addToProblem(
      problem, node_collection.getLocalParameterization());

  // Create submap registration cost function
  ceres::CostFunction* cost_function;
  if (config_.registration.jacobian_evaluation_method ==
      RegistrationCostFunction::JacobianEvaluationMethod::kNumeric) {
    cost_function = nullptr;
    LOG(FATAL) << "Numeric cost not yet implemented";
  } else {
    cost_function = new RegistrationCostFunction(config_.first_submap_ptr,
                                                 config_.second_submap_ptr,
                                                 config_.registration);
  }

  // Add the constraint to the optimization and keep track of it
  residual_block_id_ = problem->AddResidualBlock(
      cost_function, loss_function,
      first_submap_node_ptr->getPosePtr()->optimizationVectorData(),
      second_submap_node_ptr->getPosePtr()->optimizationVectorData());
}
}  // namespace voxgraph
