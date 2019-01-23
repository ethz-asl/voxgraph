//
// Created by victor on 17.01.19.
//

#include "voxgraph/pose_graph/node/submap_node.h"
#include <utility>

namespace voxgraph {
SubmapNode::SubmapNode(NodeId node_id, SubmapNode::Config config)
    : Node(node_id), config_(std::move(config)) {
  // Set the node's pose to the initial submap pose
  voxblox::Transformation::Vector6 T_vec = config.initial_submap_pose.log();
  world_t_world__node_pose_[0] = T_vec[0];
  world_t_world__node_pose_[1] = T_vec[1];
  world_t_world__node_pose_[2] = T_vec[2];
  // Indicate whether the pose should be optimized or kept constant
  constant_ = config_.set_constant;
}

const voxblox::Transformation SubmapNode::getSubmapPose() const {
  voxblox::Transformation::Vector6 T_vec = config_.initial_submap_pose.log();
  T_vec[0] = world_t_world__node_pose_[0];
  T_vec[1] = world_t_world__node_pose_[1];
  T_vec[2] = world_t_world__node_pose_[2];
  return voxblox::Transformation::exp(T_vec);
}
}  // namespace voxgraph
