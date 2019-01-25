//
// Created by victor on 04.12.18.
//

#include "voxgraph/submap_registration/submap_registerer.h"
#include <voxblox/interpolator/interpolator.h>
#include <utility>
#include "voxgraph/submap_registration/registration_cost_function_xyz.h"
#include "voxgraph/submap_registration/registration_cost_function_xyz_yaw.h"

namespace voxgraph {
SubmapRegisterer::SubmapRegisterer(
    cblox::SubmapCollection<VoxgraphSubmap>::ConstPtr submap_collection_ptr,
    const Options &options)
    : submap_collection_ptr_(std::move(submap_collection_ptr)),
      options_(options) {}

bool SubmapRegisterer::testRegistration(
    const cblox::SubmapID &reference_submap_id,
    const cblox::SubmapID &reading_submap_id, double *world_pose_reading,
    ceres::Solver::Summary *summary) {
  // Get shared pointers to the reference and reading submaps
  VoxgraphSubmap::ConstPtr reference_submap_ptr =
      submap_collection_ptr_->getSubMapConstPtrById(reference_submap_id);
  VoxgraphSubmap::ConstPtr reading_submap_ptr =
      submap_collection_ptr_->getSubMapConstPtrById(reading_submap_id);
  CHECK_NOTNULL(reference_submap_ptr);
  CHECK_NOTNULL(reading_submap_ptr);

  // Create problem and initial conditions
  ceres::Problem problem;
  ceres::LossFunction *loss_function = nullptr;

  // Get initial pose of reference submap (not touched by the optimization)
  // TODO(victorr): Clean up this entire file
  voxblox::Transformation::Vector6 T_vec_ref =
      reference_submap_ptr->getPose().log();
  double world_pose_ref[4] = {T_vec_ref[0], T_vec_ref[1], T_vec_ref[2],
                              T_vec_ref[5]};
  // Add the parameter blocks to the optimization
  if (options_.param.optimize_yaw) {
    problem.AddParameterBlock(world_pose_ref, 4);
    problem.SetParameterBlockConstant(world_pose_ref);
    problem.AddParameterBlock(world_pose_reading, 4);
  } else {
    problem.AddParameterBlock(world_pose_ref, 3);
    problem.SetParameterBlockConstant(world_pose_ref);
    problem.AddParameterBlock(world_pose_reading, 3);
  }

  // Create and add submap alignment cost function
  ceres::CostFunction *cost_function;
  if (options_.cost.cost_function_type ==
      Options::CostFunction::Type::kNumeric) {
    // Create cost function with one residual per voxel
    if (options_.param.optimize_yaw) {
      RegistrationCostFunctionXYZYaw *analytic_cost_function_ptr =
          new RegistrationCostFunctionXYZYaw(reference_submap_ptr,
                                             reading_submap_ptr, options_.cost);
      cost_function = new ceres::NumericDiffCostFunction<
          RegistrationCostFunctionXYZYaw, ceres::CENTRAL,
          ceres::DYNAMIC /* residuals */, 4 /* translation variables */>(
          analytic_cost_function_ptr, ceres::TAKE_OWNERSHIP,
          static_cast<int>(analytic_cost_function_ptr->getNumRelevantVoxels()));
    } else {
      RegistrationCostFunctionXYZ *analytic_cost_function_ptr =
          new RegistrationCostFunctionXYZ(reference_submap_ptr,
                                          reading_submap_ptr, options_.cost);
      cost_function = new ceres::NumericDiffCostFunction<
          RegistrationCostFunctionXYZ, ceres::CENTRAL,
          ceres::DYNAMIC /* residuals */, 3 /* translation variables */>(
          analytic_cost_function_ptr, ceres::TAKE_OWNERSHIP,
          static_cast<int>(analytic_cost_function_ptr->getNumRelevantVoxels()));
    }
  } else {
    if (options_.param.optimize_yaw) {
      cost_function = new RegistrationCostFunctionXYZYaw(
          reference_submap_ptr, reading_submap_ptr, options_.cost);
    } else {
      cost_function = new RegistrationCostFunctionXYZ(
          reference_submap_ptr, reading_submap_ptr, options_.cost);
    }
  }
  problem.AddResidualBlock(cost_function, loss_function, world_pose_ref,
                           world_pose_reading);

  // Run the solver
  ceres::Solver::Options ceres_options = options_.solver;
  ceres::Solve(ceres_options, &problem, summary);

  return summary->IsSolutionUsable();
  }
  }  // namespace voxgraph
