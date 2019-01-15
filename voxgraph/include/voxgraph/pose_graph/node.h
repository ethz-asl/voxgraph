//
// Created by victor on 15.01.19.
//

#ifndef VOXGRAPH_POSE_GRAPH_NODE_H_
#define VOXGRAPH_POSE_GRAPH_NODE_H_

#include <cblox/core/common.h>

namespace voxgraph {
struct Node {
  const unsigned int node_id;
  cblox::SubmapID submap_id;
  voxblox::Transformation node_pose;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_POSE_GRAPH_NODE_H_
