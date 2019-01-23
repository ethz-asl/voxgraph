//
// Created by victor on 16.01.19.
//

#ifndef VOXGRAPH_POSE_GRAPH_NODE_SUBMAP_NODE_H_
#define VOXGRAPH_POSE_GRAPH_NODE_SUBMAP_NODE_H_

#include <cblox/core/common.h>
#include <memory>
#include "voxgraph/pose_graph/node/node.h"

namespace voxgraph {
class SubmapNode : public Node {
 public:
  typedef std::shared_ptr<SubmapNode> Ptr;
  struct Config : Node::Config {
    cblox::SubmapID submap_id;
    voxblox::Transformation initial_submap_pose;
  };

  SubmapNode(NodeId node_id, Config config);

  const cblox::SubmapID getSubmapId() const { return config_.submap_id; }

  const voxblox::Transformation getSubmapPose() const;

 private:
  Config config_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_POSE_GRAPH_NODE_SUBMAP_NODE_H_
