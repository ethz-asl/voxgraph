#ifndef VOXGRAPH_FRONTEND_POSE_GRAPH_INTERFACE_NODE_TEMPLATES_H_
#define VOXGRAPH_FRONTEND_POSE_GRAPH_INTERFACE_NODE_TEMPLATES_H_

#include "voxgraph/backend/node/reference_frame_node.h"
#include "voxgraph/backend/node/submap_node.h"

namespace voxgraph {
class NodeTemplates {
 public:
  enum ReferenceFrames : ReferenceFrameNode::FrameId {
    kMissionFrame,
    kGpsFrame
  };

  NodeTemplates();

  ReferenceFrameNode::Config getReferenceFrameConfigById(
      ReferenceFrameNode::FrameId frame_id);

  const ReferenceFrameNode::Config& mission_frame;
  const ReferenceFrameNode::Config& gps_frame;
  const SubmapNode::Config& submap;

 private:
  ReferenceFrameNode::Config mission_frame_;
  ReferenceFrameNode::Config gps_frame_;
  SubmapNode::Config submap_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_FRONTEND_POSE_GRAPH_INTERFACE_NODE_TEMPLATES_H_
