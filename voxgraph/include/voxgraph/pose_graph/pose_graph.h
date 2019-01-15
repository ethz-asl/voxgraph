//
// Created by victor on 15.01.19.
//

#ifndef VOXGRAPH_POSE_GRAPH_POSE_GRAPH_H_
#define VOXGRAPH_POSE_GRAPH_POSE_GRAPH_H_

#include <vector>
#include "voxgraph/pose_graph/constraint.h"
#include "voxgraph/pose_graph/node.h"

namespace voxgraph {
class PoseGraph {
 public:
  void addNode(Node node) { nodes_.emplace_back(node); }

  void addConstraint(Constraint constraint) {
    constraints_.emplace_back(constraint);
  }

  void solve() {
    // TODO(victorr): Construct the problem
    // TODO(victorr): Add all nodes and constraints
    // TODO(victorr): Solve the problem and update the nodes_ poses
  }

 private:
  std::vector<Node> nodes_;
  std::vector<Constraint> constraints_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_POSE_GRAPH_POSE_GRAPH_H_
