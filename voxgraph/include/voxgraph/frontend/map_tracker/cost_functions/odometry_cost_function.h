#ifndef VOXGRAPH_FRONTEND_MAP_TRACKER_ODOMETRY_COST_FUNCTION_H_
#define VOXGRAPH_FRONTEND_MAP_TRACKER_ODOMETRY_COST_FUNCTION_H_

#include <Eigen/Eigen>
#include <ceres/ceres.h>

namespace voxgraph {
class OdometryCostFunction {
 public:
  OdometryCostFunction(const Eigen::Vector3d& t_S_O_prior,
                           const Eigen::Quaterniond& q_S_O_prior,
                           const Eigen::Matrix<double, 6, 6>& sqrt_information)
      : t_S_O_prior_(t_S_O_prior),
        q_S_O_prior_(q_S_O_prior),
        sqrt_information_(sqrt_information) {}

  template <typename T>
  bool operator()(const T* const t_S_O_estimate_ptr,
                  const T* const q_S_O_estimate_ptr,
                  T* residuals_ptr) const {
    // Wrap the data pointers with Eigen types
    Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_S_O_estimate(t_S_O_estimate_ptr);
    Eigen::Map<const Eigen::Quaternion<T>> q_S_O_estimate(q_S_O_estimate_ptr);

    // Compute the error between the orientation estimate and its prior
    Eigen::Quaternion<T> delta_q =
        q_S_O_prior_.template cast<T>() * q_S_O_estimate.conjugate();

    // Compute the residuals.
    // [ position         ]   [ delta_p          ]
    // [ orientation (3x1)] = [ 2 * delta_q(0:2) ]
    residuals.template block<3, 1>(0, 0) =
        t_S_O_estimate - t_S_O_prior_.template cast<T>();
    residuals.template block<3, 1>(3, 0) = T(2.0) * delta_q.vec();

    // Scale the residuals by the measurement uncertainty.
    residuals.applyOnTheLeft(sqrt_information_.template cast<T>());
    return true;
  }

  static ceres::CostFunction* Create(
      const Eigen::Vector3d& t_S_O_prior,
      const Eigen::Quaterniond& q_S_O_prior,
      const Eigen::Matrix<double, 6, 6>& sqrt_information) {
    return new ceres::AutoDiffCostFunction<OdometryCostFunction, 6, 3, 4>(
        new OdometryCostFunction(t_S_O_prior, q_S_O_prior, sqrt_information));
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  const Eigen::Vector3d t_S_O_prior_;
  const Eigen::Quaterniond q_S_O_prior_;

  const Eigen::Matrix<double, 6, 6> sqrt_information_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_FRONTEND_MAP_TRACKER_ODOMETRY_COST_FUNCTION_H_
