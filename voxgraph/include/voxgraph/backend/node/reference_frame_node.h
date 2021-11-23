#ifndef VOXGRAPH_BACKEND_NODE_REFERENCE_FRAME_NODE_H_
#define VOXGRAPH_BACKEND_NODE_REFERENCE_FRAME_NODE_H_

#include <memory>

#include "voxgraph/backend/node/node.h"

namespace voxgraph {
class ReferenceFrameNode : public Node {
 public:
  typedef std::shared_ptr<ReferenceFrameNode> Ptr;
  typedef unsigned int FrameId;

  struct Config : Node::Config {
    FrameId reference_frame_id;
  };

  ReferenceFrameNode(const NodeId& node_id, const Config& config)
      : Node(node_id, config), config_(config) {}

  FrameId getReferenceFrameId() const { return config_.reference_frame_id; }

 private:
  Config config_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_BACKEND_NODE_REFERENCE_FRAME_NODE_H_
