//
// Created by victor on 04.04.19.
//

#include "voxgraph/backend/node/node_collection.h"
#include <memory>

namespace voxgraph {
void NodeCollection::addSubmapNode(const SubmapNode::Config &config) {
  auto submap_node_ptr =
      std::make_shared<SubmapNode>(newConstraintId(), config);
  submap_nodes_.emplace(config.submap_id, submap_node_ptr);
}

void NodeCollection::addReferenceFrameNode(
    const ReferenceFrameNode::Config &config) {
  auto reference_frame_node_ptr =
      std::make_shared<ReferenceFrameNode>(newConstraintId(), config);
  reference_frame_nodes_.emplace(config.reference_frame_id,
                                 reference_frame_node_ptr);
}

SubmapNode::Ptr NodeCollection::getNodePtrBySubmapId(
    cblox::SubmapID submap_id) const {
  auto it = submap_nodes_.find(submap_id);
  if (it != submap_nodes_.end()) {
    return it->second;
  } else {
    return nullptr;
  }
}

ReferenceFrameNode::Ptr NodeCollection::getNodePtrByReferenceFrameId(
    voxgraph::ReferenceFrameNode::ReferenceFrameId reference_frame_id) const {
  auto it = reference_frame_nodes_.find(reference_frame_id);
  if (it != reference_frame_nodes_.end()) {
    return it->second;
  } else {
    return nullptr;
  }
}
}  // namespace voxgraph
