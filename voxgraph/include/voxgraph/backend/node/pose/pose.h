#ifndef VOXGRAPH_BACKEND_NODE_POSE_POSE_H_
#define VOXGRAPH_BACKEND_NODE_POSE_POSE_H_

#include "voxgraph/common.h"

namespace voxgraph {
class Pose {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef double ValueType;

  explicit Pose(const Transformation& initial_pose) {
    initial_pose_vec_ = initial_pose.log();
  }

  ~Pose() = default;

  virtual operator Transformation() const = 0;  // NOLINT

  virtual ValueType* optimizationVectorData() = 0;
  virtual size_t optimizationVectorSize() const = 0;

 protected:
  Transformation::Vector6 initial_pose_vec_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_BACKEND_NODE_POSE_POSE_H_
