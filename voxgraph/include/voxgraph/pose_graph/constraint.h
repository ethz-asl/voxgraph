//
// Created by victor on 15.01.19.
//

#ifndef VOXGRAPH_POSE_GRAPH_CONSTRAINT_H_
#define VOXGRAPH_POSE_GRAPH_CONSTRAINT_H_

#include <ceres/ceres.h>
#include <memory>
#include "voxgraph/pose_graph/node.h"

namespace voxgraph {
class Constraint {
 public:
  typedef std::shared_ptr<Constraint> Ptr;
  typedef unsigned int ConstraintId;

  struct Endpoints {
    Node::NodeId first_node_id;
    Node::NodeId second_node_id;
  };

  explicit Constraint(Endpoints internal_config)
      : endpoints_(internal_config) {}
  virtual ~Constraint() = default;

  virtual void addToProblem(const Node::NodeMap& node_map,
                            ceres::Problem* problem) = 0;

 protected:
  const Endpoints endpoints_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_POSE_GRAPH_CONSTRAINT_H_
