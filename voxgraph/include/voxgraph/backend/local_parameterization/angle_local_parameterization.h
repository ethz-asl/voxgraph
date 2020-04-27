// NOTE: This function comes directly from the ceres::examples

#ifndef VOXGRAPH_BACKEND_LOCAL_PARAMETERIZATION_ANGLE_LOCAL_PARAMETERIZATION_H_
#define VOXGRAPH_BACKEND_LOCAL_PARAMETERIZATION_ANGLE_LOCAL_PARAMETERIZATION_H_

#include <ceres/ceres.h>

#include "voxgraph/backend/local_parameterization/normalize_angle.h"

namespace voxgraph {
// Define a local parameterization that keeps the angle within [-pi to pi) rad.
class AngleLocalParameterization {
 public:
  template <typename T>
  bool operator()(const T* theta_radians, const T* delta_theta_radians,
                  T* theta_radians_plus_delta) const {
    *theta_radians_plus_delta =
        NormalizeAngle(*theta_radians + *delta_theta_radians);

    return true;
  }

  static ceres::LocalParameterization* Create() {
    return (new ceres::AutoDiffLocalParameterization<AngleLocalParameterization,
                                                     1, 1>);
  }
};
}  // namespace voxgraph

#endif  // VOXGRAPH_BACKEND_LOCAL_PARAMETERIZATION_ANGLE_LOCAL_PARAMETERIZATION_H_
