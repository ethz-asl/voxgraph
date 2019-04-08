//
// Created by victor on 04.04.19.
//

#ifndef VOXGRAPH_BACKEND_CONSTRAINT_COST_FUNCTIONS_RELATIVE_POSE_COST_FUNCTION_INL_H_
#define VOXGRAPH_BACKEND_CONSTRAINT_COST_FUNCTIONS_RELATIVE_POSE_COST_FUNCTION_INL_H_

#include "voxgraph/backend/normalize_angle.h"

namespace voxgraph {
template <typename T>
bool RelativePoseCostFunction::operator()(const T *const pose_A,
                                          const T *const pose_B,
                                          T *residuals) const {
  const Eigen::Matrix<T, 3, 1> t_world_A(pose_A[0], pose_A[1], pose_A[2]);
  const Eigen::Matrix<T, 3, 1> t_world_B(pose_B[0], pose_B[1], pose_B[2]);
  const T yaw_world_A(pose_A[3]);
  const T yaw_world_B(pose_B[3]);

  Eigen::Map<Eigen::Matrix<T, 4, 1>> residuals_map(residuals);

  // Compute translation error
  residuals_map.template head<3>() =
      rotationMatrixFromYaw(static_cast<T>(observed_relative_yaw_)) *
          (t_world_B - t_world_A) -
      observed_relative_translation_.cast<T>();

  // Compute yaw error and normalize the angle
  residuals_map(3) = NormalizeAngle((yaw_world_B - yaw_world_A) -
                                    static_cast<T>(observed_relative_yaw_));

  // Scale the residuals by the square root information matrix to account for
  // the measurement uncertainty
  residuals_map = sqrt_information_matrix_.cast<T>() * residuals_map;

  return true;
}

template <typename T>
Eigen::Matrix<T, 3, 3> RelativePoseCostFunction::rotationMatrixFromYaw(
    T yaw_radians) const {
  const T cos_yaw = ceres::cos(yaw_radians);
  const T sin_yaw = ceres::sin(yaw_radians);

  Eigen::Matrix<T, 3, 3> rotation_matrix;
  rotation_matrix << cos_yaw, -sin_yaw, 0, sin_yaw, cos_yaw, 0, 0, 0, 1;
  return rotation_matrix;
}
}  // namespace voxgraph

#endif  // VOXGRAPH_BACKEND_CONSTRAINT_COST_FUNCTIONS_RELATIVE_POSE_COST_FUNCTION_INL_H_
