//
// Created by victor on 18.01.19.
//

#ifndef VOXGRAPH_POSE_GRAPH_NODE_COLLECTION_H_
#define VOXGRAPH_POSE_GRAPH_NODE_COLLECTION_H_

#include <map>
#include <memory>
#include "voxgraph/pose_graph/node/submap_node.h"

namespace voxgraph {
class NodeCollection {
 public:
  typedef std::map<cblox::SubmapID, SubmapNode::Ptr> SubmapNodeMap;

  void addNode(const SubmapNode::Config& config) {
    auto ptr = std::make_shared<SubmapNode>(newNodeId(), config);
    submap_nodes_.emplace(config.submap_id, ptr);
  }

  SubmapNode::Ptr getNodePtrById(cblox::SubmapID submap_id) const {
    auto it = submap_nodes_.find(submap_id);
    if (it != submap_nodes_.end()) {
      return it->second;
    } else {
      return nullptr;
    }
  }

  const SubmapNodeMap& getSubmapNodes() { return submap_nodes_; }

 private:
  Node::NodeId node_id_counter_ = 0;
  const Node::NodeId newNodeId() { return node_id_counter_++; }

  SubmapNodeMap submap_nodes_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_POSE_GRAPH_NODE_COLLECTION_H_
