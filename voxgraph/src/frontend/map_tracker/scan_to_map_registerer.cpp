#include <thread>
#include "voxgraph/frontend/map_tracker/scan_to_map_registerer.h"
#include "voxgraph/frontend/map_tracker/cost_functions/odometry_cost_function.h"

namespace voxgraph {
bool ScanToMapRegisterer::refineSensorPose(
    const sensor_msgs::PointCloud2::Ptr &pointcloud_msg,
    const Transformation &T_mission__sensor_prior,
    Transformation *T_mission__sensor_refined) const {
  CHECK_NOTNULL(T_mission__sensor_refined);

  // NOTE: The frame convention is:
  //       - M: Mission
  //       - S: Submap
  //       - C: Sensor
  // TODO(victorr): Update the code below s.t. the odom_information matrix is
  //                applied in odom frame (currently in sensor frame)

  // Get a pointer to the submap we track and convert the pose into its frame
  const SubmapID tracked_submap_id =
      submap_collection_ptr_->getActiveSubmapID();
  VoxgraphSubmap::ConstPtr tracked_submap_ptr =
      submap_collection_ptr_->getSubmapConstPtr(tracked_submap_id);
  Transformation T_M_S;
  CHECK(submap_collection_ptr_->getSubmapPose(tracked_submap_id, &T_M_S));
  const Transformation T_S_C_prior = T_M_S.inverse() * T_mission__sensor_prior;

  // Create Ceres problem
  ceres::Problem::Options problem_options;
  problem_options.local_parameterization_ownership =
      ceres::DO_NOT_TAKE_OWNERSHIP;
  problem_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres::Problem problem(problem_options);
  // TODO(victorr): Evaluate this
  ceres::Solver::Options solver_options;
  solver_options.parameter_tolerance = 3e-3;
  solver_options.max_num_iterations = 2;
  solver_options.max_solver_time_in_seconds = 0.08;
  solver_options.num_threads = 2;
  //  solver_options.linear_solver_type = ceres::LinearSolverType::DENSE_QR;
  solver_options.linear_solver_type =
      ceres::LinearSolverType::DENSE_NORMAL_CHOLESKY;
  ceres::Solver::Summary solver_summary;

  // Initialize and add the translation parameter block
  Eigen::Vector3d t_S_C_refined =
      T_S_C_prior.getPosition().cast<double>();
  problem.AddParameterBlock(t_S_C_refined.data(), 3);

  // Initialize and add the rotation parameter block
  // NOTE: An Eigen quaternion local parameterization is used s.t. Ceres takes
  //       into account that the quaternion coefficients should remain on the
  //       quaternion's unit length manifold during the optimization
  Eigen::Quaterniond q_S_C_refined =
      T_S_C_prior.getEigenQuaternion().cast<double>();
  ceres::EigenQuaternionParameterization eigen_quaternion_parameterization;
  problem.AddParameterBlock(q_S_C_refined.coeffs().data(), 4);
  problem.SetParameterization(q_S_C_refined.coeffs().data(),
                              &eigen_quaternion_parameterization);

  // TODO(victorr): Instead of the above, use Pose6D to elegantly go back and
  //                forth between voxblox::Transformations and Eigen types

  // Create and add the relative pose cost function
  Eigen::Matrix<double, 6, 6> odom_sqrt_information;
  odom_sqrt_information.setIdentity();
  // for penguin
  odom_sqrt_information.topLeftCorner(3, 3) *= 1e5;      // translation xyz
  odom_sqrt_information.bottomRightCorner(3, 3) *= 1e7;  // rotation rpy
  // for anymal
  //  odom_sqrt_information(0, 0) *= 1e3;  // translation x
  //  odom_sqrt_information(1, 1) *= 1e3;  // translation y
  //  odom_sqrt_information(2, 2) *= 1e2;  // translation z
  //  odom_sqrt_information(3, 3) *= 1e7;  // roll
  //  odom_sqrt_information(4, 4) *= 1e7;  // pitch
  //  odom_sqrt_information(5, 5) *= 1e5;  // yaw
  ceres::CostFunction* odom_cost_function = OdometryCostFunction::Create(
      T_S_C_prior.getPosition().cast<double>(),
      T_S_C_prior.getEigenQuaternion().cast<double>(),
      odom_sqrt_information);
  problem.AddResidualBlock(odom_cost_function,
                           nullptr,
                           t_S_C_refined.data(),
                           q_S_C_refined.coeffs().data());

  // Create and add the scan-to-map registration cost function
  ceres::CostFunction* registration_cost_function =
      ScanRegistrationCostFunction::Create(pointcloud_msg, tracked_submap_ptr);
  problem.AddResidualBlock(registration_cost_function, nullptr,
                           t_S_C_refined.data(), q_S_C_refined.coeffs().data());

  // Run the solver
  ceres::Solve(solver_options, &problem, &solver_summary);

  // Return solution
  if (solver_summary.IsSolutionUsable()) {
    // Extract the refined pose
    Transformation T_S_C_refined;
    T_S_C_refined.getPosition() = t_S_C_refined.cast<voxblox::FloatingPoint>();
    T_S_C_refined.getRotation() =
        Transformation::Rotation(q_S_C_refined.cast<voxblox::FloatingPoint>());
    *T_mission__sensor_refined = T_M_S * T_S_C_refined;

    // Print runtime stats
    if (verbose_) {
      printf(
          "-- voxgraph icp: reduced cost by %.2f%% from %.0fe3 to %.0fe3 "
          "in %.0fms (%.0u iterations)\n",
          (solver_summary.initial_cost - solver_summary.final_cost) /
              solver_summary.initial_cost * 100,
          solver_summary.initial_cost / 1000, solver_summary.final_cost / 1000,
          solver_summary.total_time_in_seconds * 1000,
          static_cast<unsigned int>(solver_summary.iterations.size()));
      Transformation::Vector6 delta_vec =
          (T_mission__sensor_prior.inverse() * (*T_mission__sensor_refined))
              .log();
      std::cout << "--> " << delta_vec.format(ioformat_) << std::endl;
    }
    return true;
  } else {
    T_mission__sensor_refined = nullptr;
    return false;
  }
}
}  // namespace voxgraph
