//
// Created by victor on 15.01.19.
//

#ifndef VOXGRAPH_POSE_GRAPH_CONSTRAINT_H_
#define VOXGRAPH_POSE_GRAPH_CONSTRAINT_H_

namespace voxgraph {
struct Constraint {
  unsigned int first_node_id;
  unsigned int second_node_id;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_POSE_GRAPH_CONSTRAINT_H_
