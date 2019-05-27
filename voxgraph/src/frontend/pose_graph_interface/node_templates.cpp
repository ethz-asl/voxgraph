#include "voxgraph/frontend/pose_graph_interface/node_templates.h"

namespace voxgraph {
NodeTemplates::NodeTemplates()
    : world_frame(world_frame_), gps_frame(gps_frame_), submap(submap_) {
  // Initialize the world frame config
  // NOTE: The frame is fixed to (X, Y, Z, Yaw) = (0, 0, 0, 0)
  world_frame_.reference_frame_id = kWorldFrame;
  world_frame_.set_constant = true;
  world_frame_.T_world_node_initial.setIdentity();

  // Initialize the GPS frame config
  // NOTE: We let the frame float freely w.r.t. the optimization's world frame.
  //       This way the submap positions will not jump when the first GPS
  //       measurement is received even if the robot started mapping before
  //       getting an absolute position fix. The absolute pose of a submap can
  //       still be retrieved by multiplying the GPS reference frame's absolute
  //       pose with the transform between the GPS frame and the submap.
  gps_frame_.reference_frame_id = kGpsFrame;
  gps_frame_.set_constant = false;
}

ReferenceFrameNode::Config NodeTemplates::getReferenceFrameConfigById(
    ReferenceFrameNode::FrameId frame_id) {
  switch (frame_id) {
    case kWorldFrame:
      return world_frame;
    case kGpsFrame:
      return gps_frame;
    default:
      LOG(FATAL) << "Requested config for unknown reference frame id: "
                 << frame_id;
      break;
  }
}
}  // namespace voxgraph
