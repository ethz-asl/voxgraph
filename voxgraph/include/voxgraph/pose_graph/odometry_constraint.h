//
// Created by victor on 16.01.19.
//

#ifndef VOXGRAPH_POSE_GRAPH_ODOMETRY_CONSTRAINT_H_
#define VOXGRAPH_POSE_GRAPH_ODOMETRY_CONSTRAINT_H_

#include <memory>
#include "voxgraph/pose_graph/constraint.h"

namespace voxgraph {
class OdometryConstraint : public Constraint {
 public:
  typedef std::shared_ptr<OdometryConstraint> Ptr;
  struct Config {};

  OdometryConstraint(ConstraintId constraint_id, Config config)
      : Constraint(constraint_id), config_(config) {}

  void addToProblem(const NodeCollection& node_collection,
                    ceres::Problem* problem) final {}

 private:
  Config config_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_POSE_GRAPH_ODOMETRY_CONSTRAINT_H_
