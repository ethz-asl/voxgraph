#ifndef VOXGRAPH_BACKEND_CONSTRAINT_COST_FUNCTIONS_RELATIVE_POSE_COST_FUNCTION_INL_H_
#define VOXGRAPH_BACKEND_CONSTRAINT_COST_FUNCTIONS_RELATIVE_POSE_COST_FUNCTION_INL_H_

#include "voxgraph/backend/local_parameterization/normalize_angle.h"

namespace voxgraph {
template <typename T>
bool RelativePoseCostFunction::operator()(const T* const pose_A,
                                          const T* const pose_B,
                                          T* residuals) const {
  const Eigen::Matrix<T, 3, 1> t_odom_A(pose_A[0], pose_A[1], pose_A[2]);
  const Eigen::Matrix<T, 3, 1> t_odom_B(pose_B[0], pose_B[1], pose_B[2]);
  const T yaw_odom_A(pose_A[3]);
  const T yaw_odom_B(pose_B[3]);
  CHECK_NEAR(yaw_odom_A, T(0), T(M_PI));
  CHECK_NEAR(yaw_odom_B, T(0), T(M_PI));

  Eigen::Map<Eigen::Matrix<T, 4, 1>> residuals_map(residuals);

  // Compute translation error
  residuals_map.template head<3>() =
      rotationMatrixFromYaw<T>(yaw_odom_A).transpose() * (t_odom_B - t_odom_A) -
      observed_relative_translation_.cast<T>();

  // Compute yaw error and normalize the angle
  residuals_map(3) = NormalizeAngle((yaw_odom_B - yaw_odom_A) -
                                    static_cast<T>(observed_relative_yaw_));

  if (verbose_) {
    std::cout << "t_odom_A: " << t_odom_A(0) << ", " << t_odom_A(1) << ", "
              << t_odom_A(2) << "\n"
              << "t_odom_B: " << t_odom_B(0) << ", " << t_odom_B(1) << ", "
              << t_odom_B(2) << "\n"
              << "Rotated(t_odom_B - t_odom_A):\n"
              << (rotationMatrixFromYaw(yaw_odom_A).transpose() *
                  (t_odom_B - t_odom_A))(0)
              << "\n"
              << (rotationMatrixFromYaw(yaw_odom_A).transpose() *
                  (t_odom_B - t_odom_A))(1)
              << "\n"
              << (rotationMatrixFromYaw(yaw_odom_A).transpose() *
                  (t_odom_B - t_odom_A))(2)
              << "\n"
              << "observed_relative_translation: "
              << observed_relative_translation_.cast<T>()(0) << ", "
              << observed_relative_translation_.cast<T>()(1) << ", "
              << observed_relative_translation_.cast<T>()(2) << "\n"
              << "yaw_odom_A: " << yaw_odom_A << "\n"
              << "yaw_odom_B: " << yaw_odom_B << "\n"
              << "observed_relative_yaw: "
              << static_cast<T>(observed_relative_yaw_) << "\n"
              << "Error:" << residuals_map(0) << ", " << residuals_map(1)
              << ", " << residuals_map(2) << ", " << residuals_map(3) << "\n"
              << std::endl;
  }

  // Scale the residuals by the square root information matrix to account for
  // the measurement uncertainty
  residuals_map = sqrt_information_matrix_.template cast<T>() * residuals_map;

  if (verbose_) {
    std::cout << "Error scaled by information: " << residuals_map(0) << ", "
              << residuals_map(1) << ", " << residuals_map(2) << ", "
              << residuals_map(3) << "\n"
              << "------------------------------------------" << std::endl;
  }

  return true;
}

template <typename T>
const Eigen::Matrix<T, 3, 3> RelativePoseCostFunction::rotationMatrixFromYaw(
    T yaw_radians) const {
  const T cos_yaw = ceres::cos(yaw_radians);
  const T sin_yaw = ceres::sin(yaw_radians);
  const T zero = static_cast<T>(0);
  const T one = static_cast<T>(1);

  Eigen::Matrix<T, 3, 3> rotation_matrix;
  rotation_matrix << cos_yaw, -sin_yaw, zero, sin_yaw, cos_yaw, zero, zero,
      zero, one;
  return rotation_matrix;
}
}  // namespace voxgraph

#endif  // VOXGRAPH_BACKEND_CONSTRAINT_COST_FUNCTIONS_RELATIVE_POSE_COST_FUNCTION_INL_H_
