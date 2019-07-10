#include <thread>
#include "voxgraph/frontend/map_tracker/scan_to_map_registerer.h"
#include "voxgraph/frontend/map_tracker/cost_functions/odometry_cost_function.h"

namespace voxgraph {
bool ScanToMapRegisterer::refineOdometry(
    const sensor_msgs::PointCloud2::Ptr &pointcloud_msg,
    const Transformation &T_submap__odometry_prior,
    Transformation *T_submap__refined_odometry) {
  CHECK_NOTNULL(T_submap__refined_odometry);

  // Get shared pointers to the reference and reading submaps
  SubmapID active_submap_id = submap_collection_ptr_->getActiveSubmapID();
  VoxgraphSubmap::ConstPtr active_submap_ptr =
      submap_collection_ptr_->getSubmapConstPtr(active_submap_id);

  // Create Ceres problem
  ceres::Problem::Options problem_options;
  problem_options.local_parameterization_ownership =
      ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres::Problem problem(problem_options);
  ceres::LossFunction *loss_function = nullptr;
  ceres::Solver::Options solver_options;
  solver_options.parameter_tolerance = 3e-4;
  solver_options.max_num_iterations = 4;
  solver_options.max_solver_time_in_seconds = 0.10;
  solver_options.num_threads = std::thread::hardware_concurrency();
//  solver_options.linear_solver_type = ceres::LinearSolverType::DENSE_SCHUR;
  ceres::Solver::Summary solver_summary;

  // Initialize and add the translation parameter block
  Eigen::Vector3d t_S_O_refined =
      T_submap__odometry_prior.getPosition().cast<double>();
  problem.AddParameterBlock(t_S_O_refined.data(), 3);

  // Initialize and add the rotation parameter block
  // NOTE: An Eigen quaternion local parameterization is used s.t. Ceres takes
  //       into account that the quaternion coefficients should remain on the
  //       quaternion's unit length manifold during the optimization
  Eigen::Quaterniond q_S_O_refined =
      T_submap__odometry_prior.getEigenQuaternion().cast<double>();
  ceres::EigenQuaternionParameterization eigen_quaternion_parameterization;
  problem.AddParameterBlock(q_S_O_refined.coeffs().data(), 4);
  problem.SetParameterization(q_S_O_refined.coeffs().data(),
                              &eigen_quaternion_parameterization);

  // TODO(victorr): Instead of the above, use Pose6D to elegantly go back and
  //                forth between voxblox::Transformations and Eigen types

  // Create and add the relative pose cost function
  Eigen::Matrix<double, 6, 6> odom_sqrt_information;
  odom_sqrt_information.setIdentity();
  odom_sqrt_information *= 10;
  ceres::CostFunction* odom_cost_function = OdometryCostFunction::Create(
      T_submap__odometry_prior.getPosition().cast<double>(),
      T_submap__odometry_prior.getEigenQuaternion().cast<double>(),
      odom_sqrt_information);
  problem.AddResidualBlock(odom_cost_function,
                           loss_function,
                           t_S_O_refined.data(),
                           q_S_O_refined.coeffs().data());

  // Create and add the scan-to-map registration cost function
  ceres::CostFunction* registration_cost_function =
      ScanRegistrationCostFunction::Create(pointcloud_msg, active_submap_ptr);
  problem.AddResidualBlock(registration_cost_function,
                           loss_function,
                           t_S_O_refined.data(),
                           q_S_O_refined.coeffs().data());

  // Run the solver
  ceres::Solve(solver_options, &problem, &solver_summary);
  std::cout << solver_summary.FullReport() << std::endl;

  // Return solution
  if (solver_summary.IsSolutionUsable()) {
    T_submap__refined_odometry->getPosition() =
        t_S_O_refined.cast<voxblox::FloatingPoint>();
    T_submap__refined_odometry->getRotation() =
        Transformation::Rotation(q_S_O_refined.cast<voxblox::FloatingPoint>());
    return true;
  } else {
    T_submap__refined_odometry = nullptr;
    return false;
  }
}
}  // namespace voxgraph
