//
// Created by victor on 03.04.19.
//

#ifndef VOXGRAPH_POSE_GRAPH_NODE_REFERENCE_FRAME_NODE_H_
#define VOXGRAPH_POSE_GRAPH_NODE_REFERENCE_FRAME_NODE_H_

#include <voxblox/core/common.h>
#include <memory>
#include "voxgraph/pose_graph/node/node.h"

namespace voxgraph {
class ReferenceFrameNode : public Node {
 public:
  typedef std::shared_ptr<ReferenceFrameNode> Ptr;
  typedef unsigned int ReferenceFrameId;

  struct Config : Node::Config {
    ReferenceFrameId reference_frame_id;
    voxblox::Transformation initial_reference_frame_pose;
  };

  ReferenceFrameNode(NodeId node_id, Config config);

  const ReferenceFrameId getReferenceFrameId() const {
    return config_.reference_frame_id;
  }

  const voxblox::Transformation getReferenceFramePose() const;

 private:
  Config config_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_POSE_GRAPH_NODE_REFERENCE_FRAME_NODE_H_
