#ifndef VOXGRAPH_BACKEND_NODE_POSE_POSE_4D_H_
#define VOXGRAPH_BACKEND_NODE_POSE_POSE_4D_H_

#include "voxgraph/backend/node/pose/pose.h"

namespace voxgraph {
class Pose4D : public Pose {
 public:
  typedef std::array<Pose::ValueType, 4> XyzYawVector;

  // Converting constructor from minkindr
  explicit Pose4D(const Transformation& initial_pose);

  // User defined conversion to minkindr
  operator Transformation() const final;  // NOLINT

  XyzYawVector::value_type* optimizationVectorData() final {
    return xyz_yaw_vector_.data();
  }

  size_t optimizationVectorSize() const final { return xyz_yaw_vector_.size(); }

 private:
  XyzYawVector xyz_yaw_vector_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_BACKEND_NODE_POSE_POSE_4D_H_
