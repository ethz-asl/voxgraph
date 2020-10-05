#include "voxgraph/backend/node/pose/pose_4d.h"

namespace voxgraph {
Pose4D::Pose4D(const Transformation& initial_pose) : Pose(initial_pose) {
  // Initialize the optimization variables
  xyz_yaw_vector_[0] = initial_pose_vec_[0];
  xyz_yaw_vector_[1] = initial_pose_vec_[1];
  xyz_yaw_vector_[2] = initial_pose_vec_[2];
  xyz_yaw_vector_[3] = initial_pose_vec_[5];
}

Pose4D::operator Transformation() const {
  Transformation::Vector6 T_vec_6d;
  // Set x, y, z and yaw from the optimization variables
  T_vec_6d[0] = xyz_yaw_vector_[0];
  T_vec_6d[1] = xyz_yaw_vector_[1];
  T_vec_6d[2] = xyz_yaw_vector_[2];
  T_vec_6d[5] = xyz_yaw_vector_[3];
  // Set pitch and roll to zero from the initial pose
  T_vec_6d[3] = initial_pose_vec_[3];
  T_vec_6d[4] = initial_pose_vec_[4];
  Transformation result = Transformation::exp(T_vec_6d);
  result.getRotation().normalize();
  return result;
}
}  // namespace voxgraph
