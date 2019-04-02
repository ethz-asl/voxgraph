//
// Created by victor on 04.04.19.
//

#ifndef VOXGRAPH_POSE_GRAPH_CONSTRAINT_COST_FUNCTIONS_RELATIVE_POSE_COST_FUNCTION_H_
#define VOXGRAPH_POSE_GRAPH_CONSTRAINT_COST_FUNCTIONS_RELATIVE_POSE_COST_FUNCTION_H_

#include <ceres/ceres.h>
#include "voxgraph/pose_graph/constraint/constraint.h"

namespace voxgraph {
class RelativePoseCostFunction {
 public:
  RelativePoseCostFunction(
      const voxblox::Transformation& observed_relative_pose,
      const Constraint::InformationMatrix& sqrt_information_matrix)
      : observed_relative_translation_(
            observed_relative_pose.getPosition().cast<double>()),
        observed_relative_yaw_(
            static_cast<double>(observed_relative_pose.log()[5])),
        sqrt_information_matrix_(sqrt_information_matrix) {}

  template <typename T>
  bool operator()(const T* pose_A, const T* pose_B, T* residuals) const;

  static ceres::CostFunction* Create(
      const voxblox::Transformation& observed_relative_pose,
      const Constraint::InformationMatrix& sqrt_information_matrix);

 private:
  Eigen::Matrix<double, 3, 1> observed_relative_translation_;
  double observed_relative_yaw_;
  Constraint::InformationMatrix sqrt_information_matrix_;

  template <typename T>
  Eigen::Matrix<T, 3, 3> rotationMatrixFromYaw(T yaw_radians) const;
};
}  // namespace voxgraph

#include "voxgraph/pose_graph/constraint/cost_functions/relative_pose_cost_function_inl.h"

#endif  // VOXGRAPH_POSE_GRAPH_CONSTRAINT_COST_FUNCTIONS_RELATIVE_POSE_COST_FUNCTION_H_
