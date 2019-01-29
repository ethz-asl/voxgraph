//
// Created by victor on 15.01.19.
//

#ifndef VOXGRAPH_POSE_GRAPH_CONSTRAINT_CONSTRAINT_H_
#define VOXGRAPH_POSE_GRAPH_CONSTRAINT_CONSTRAINT_H_

#include <ceres/ceres.h>
#include <memory>
#include "voxgraph/pose_graph/node_collection.h"

namespace voxgraph {
class Constraint {
 public:
  typedef std::shared_ptr<Constraint> Ptr;
  typedef unsigned int ConstraintId;

  explicit Constraint(ConstraintId constraint_id)
      : constraint_id_(constraint_id) {}
  virtual ~Constraint() = default;

  virtual void addToProblem(const NodeCollection &node_collection,
                            ceres::Problem *problem) = 0;

  const ceres::ResidualBlockId getResidualBlockId() {
    return residual_block_id_;
  }

 protected:
  const ConstraintId constraint_id_;
  ceres::ResidualBlockId residual_block_id_ = nullptr;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_POSE_GRAPH_CONSTRAINT_CONSTRAINT_H_
