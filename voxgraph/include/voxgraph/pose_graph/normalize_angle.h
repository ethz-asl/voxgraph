//
// Created by victor on 05.04.19.
// This function comes directly from the ceres::examples
//

#ifndef VOXGRAPH_POSE_GRAPH_NORMALIZE_ANGLE_H_
#define VOXGRAPH_POSE_GRAPH_NORMALIZE_ANGLE_H_

#include <ceres/ceres.h>

namespace voxgraph {
// Normalizes the angle in radians between [-pi and pi).
template <typename T>
inline T NormalizeAngle(const T& angle_radians) {
  // Use ceres::floor because it is specialized for double and Jet types.
  T two_pi(2.0 * M_PI);
  return angle_radians -
         two_pi * ceres::floor((angle_radians + T(M_PI)) / two_pi);
}
}  // namespace voxgraph

#endif  // VOXGRAPH_POSE_GRAPH_NORMALIZE_ANGLE_H_
