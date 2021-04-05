#ifndef VOXGRAPH_BACKEND_NODE_SUBMAP_NODE_H_
#define VOXGRAPH_BACKEND_NODE_SUBMAP_NODE_H_

#include <memory>

#include <cblox/core/common.h>

#include "voxgraph/backend/node/node.h"

namespace voxgraph {
class SubmapNode : public Node {
 public:
  typedef std::shared_ptr<SubmapNode> Ptr;
  typedef cblox::SubmapID SubmapId;

  struct Config : Node::Config {
    SubmapId submap_id;
  };

  SubmapNode(const NodeId& node_id, const Config& config)
      : Node(node_id, config), config_(config) {}

  cblox::SubmapID getSubmapId() const { return config_.submap_id; }

 private:
  Config config_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_BACKEND_NODE_SUBMAP_NODE_H_
