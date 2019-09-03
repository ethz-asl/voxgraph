#ifndef VOXGRAPH_BACKEND_NODE_POSE_POSE_6D_H_
#define VOXGRAPH_BACKEND_NODE_POSE_POSE_6D_H_

#include "voxgraph/backend/node/pose/pose.h"

namespace voxgraph {
class Pose6D : public Pose {
 public:
  typedef std::array<Pose::ValueType, 6> R3So3Vector;

  // Converting constructor from minkindr
  explicit Pose6D(const Transformation& initial_pose);

  // User defined conversion to minkindr
  operator Transformation() const final;  // NOLINT

  R3So3Vector::value_type* optimizationVectorData() final {
    return r3_so3_vector_.data();
  }

  size_t optimizationVectorSize() const final { return r3_so3_vector_.size(); }

 private:
  R3So3Vector r3_so3_vector_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_BACKEND_NODE_POSE_POSE_6D_H_
