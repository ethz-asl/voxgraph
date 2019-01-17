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

  explicit OdometryConstraint(Config config, Endpoints endpoints)
      : config_(config), Constraint(endpoints) {}

  void addToProblem(const Node::NodeMap& node_map,
                    ceres::Problem* problem) final {}

 private:
  Config config_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_POSE_GRAPH_ODOMETRY_CONSTRAINT_H_
