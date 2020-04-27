#ifndef VOXGRAPH_BACKEND_CONSTRAINT_ABSOLUTE_POSE_CONSTRAINT_H_
#define VOXGRAPH_BACKEND_CONSTRAINT_ABSOLUTE_POSE_CONSTRAINT_H_

#include <memory>

#include "voxgraph/backend/constraint/constraint.h"

namespace voxgraph {
class AbsolutePoseConstraint : public Constraint {
 public:
  typedef std::shared_ptr<AbsolutePoseConstraint> Ptr;
  struct Config : Constraint::Config {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ReferenceFrameNode::FrameId reference_frame_id;
    cblox::SubmapID submap_id;
    voxblox::Transformation T_ref_submap;
  };

  AbsolutePoseConstraint(ConstraintId constraint_id, const Config& config)
      : Constraint(constraint_id, config), config_(config) {}

  void addToProblem(const NodeCollection& node_collection,
                    ceres::Problem* problem) final;

 private:
  const Config config_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_BACKEND_CONSTRAINT_ABSOLUTE_POSE_CONSTRAINT_H_
