#include "voxgraph/backend/node/pose/pose_6d.h"

namespace voxgraph {
Pose6D::Pose6D(const Transformation& initial_pose) : Pose(initial_pose) {
  // Initialize the optimization variables
  for (int i = 0; i < 6; i++) {
    r3_so3_vector_[i] = initial_pose_vec_[i];
  }
}

Pose6D& Pose6D::operator=(const Transformation& rhs) {
  Transformation::Vector6 rhs_log = rhs.log();
  for (int i = 0; i < 6; i++) {
    r3_so3_vector_[i] = rhs_log[i];
  }

  return *this;
}

Pose6D::operator Transformation() const {
  // Convert the optimization array to a minkindr compatible type
  Transformation::Vector6 T_vec_6d;
  for (int i = 0; i < 6; i++) {
    T_vec_6d[i] = r3_so3_vector_[i];
  }

  return Transformation::exp(T_vec_6d);
}
}  // namespace voxgraph
