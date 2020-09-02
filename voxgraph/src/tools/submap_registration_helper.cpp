#include "voxgraph/tools/submap_registration_helper.h"

#include <utility>

#include <voxblox/interpolator/interpolator.h>

#include "voxgraph/backend/constraint/cost_functions/registration_cost_function.h"

namespace voxgraph {
SubmapRegistrationHelper::SubmapRegistrationHelper(
    cblox::SubmapCollection<VoxgraphSubmap>::ConstPtr submap_collection_ptr,
    const Options& options)
    : submap_collection_ptr_(std::move(submap_collection_ptr)),
      options_(options) {}

bool SubmapRegistrationHelper::testRegistration(
    const cblox::SubmapID& reference_submap_id,
    const cblox::SubmapID& reading_submap_id, double* odom_pose_reading,
    ceres::Solver::Summary* summary) {
  // Get shared pointers to the reference and reading submaps
  VoxgraphSubmap::ConstPtr reference_submap_ptr =
      submap_collection_ptr_->getSubmapConstPtr(reference_submap_id);
  VoxgraphSubmap::ConstPtr reading_submap_ptr =
      submap_collection_ptr_->getSubmapConstPtr(reading_submap_id);
  CHECK_NOTNULL(reference_submap_ptr);
  CHECK_NOTNULL(reading_submap_ptr);

  // Create problem and initial conditions
  ceres::Problem problem;
  ceres::LossFunction* loss_function = nullptr;  // No robust loss function

  // Get initial pose of reference submap (not touched by the optimization)
  voxblox::Transformation::Vector6 T_vec_ref =
      reference_submap_ptr->getPose().log();
  double odom_pose_ref[4] = {T_vec_ref[0], T_vec_ref[1], T_vec_ref[2],
                             T_vec_ref[5]};

  // Add the parameter blocks to the optimization
  problem.AddParameterBlock(odom_pose_ref, 4);
  problem.SetParameterBlockConstant(odom_pose_ref);
  problem.AddParameterBlock(odom_pose_reading, 4);

  // Create the submap registration cost function
  RegistrationCostFunction* registration_cost_function =
      new RegistrationCostFunction(reference_submap_ptr, reading_submap_ptr,
                                   options_.registration);

  // Toggle between analytic and numeric Jacobians
  ceres::CostFunction* ceres_cost_function;
  if (options_.registration.jacobian_evaluation_method ==
      RegistrationCostFunction::JacobianEvaluationMethod::kNumeric) {
    // Wrap the registration cost function in a numeric diff cost function,
    // which only requests residuals and calculates the Jacobians numerically
    ceres_cost_function = new ceres::NumericDiffCostFunction<
        RegistrationCostFunction, ceres::CENTRAL, ceres::DYNAMIC, 4, 4>(
        registration_cost_function, ceres::TAKE_OWNERSHIP,
        registration_cost_function->num_residuals());
  } else {
    // Let Ceres use the registration cost function's analytic Jacobians
    ceres_cost_function = registration_cost_function;
  }

  // Add the cost function to the problem
  problem.AddResidualBlock(ceres_cost_function, loss_function, odom_pose_ref,
                           odom_pose_reading);

  // Run the solver
  ceres::Solve(options_.solver, &problem, summary);

  return summary->IsSolutionUsable();
}
}  // namespace voxgraph
