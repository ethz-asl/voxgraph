#include "voxgraph/backend/node/node_collection.h"

#include <memory>

#include "voxgraph/backend/local_parameterization/angle_local_parameterization.h"

namespace voxgraph {
NodeCollection::NodeCollection() {
  local_parameterization_ = std::make_shared<ceres::ProductParameterization>(
      new ceres::IdentityParameterization(3),
      AngleLocalParameterization::Create());
}

void NodeCollection::addSubmapNode(const SubmapNode::Config& config) {
  auto submap_node_ptr = std::make_shared<SubmapNode>(newNodeId(), config);
  submap_nodes_.emplace(config.submap_id, submap_node_ptr);
}

void NodeCollection::addReferenceFrameNode(
    const ReferenceFrameNode::Config& config) {
  auto reference_frame_node_ptr =
      std::make_shared<ReferenceFrameNode>(newNodeId(), config);
  reference_frame_nodes_.emplace(config.reference_frame_id,
                                 reference_frame_node_ptr);
}

SubmapNode::Ptr NodeCollection::getSubmapNodePtrById(
    const cblox::SubmapID& submap_id) const {
  auto it = submap_nodes_.find(submap_id);
  if (it != submap_nodes_.end()) {
    return it->second;
  } else {
    return nullptr;
  }
}

ReferenceFrameNode::Ptr NodeCollection::getReferenceFrameNodePtrById(
    const ReferenceFrameNode::FrameId& frame_id) const {
  auto it = reference_frame_nodes_.find(frame_id);
  if (it != reference_frame_nodes_.end()) {
    return it->second;
  } else {
    return nullptr;
  }
}
}  // namespace voxgraph
