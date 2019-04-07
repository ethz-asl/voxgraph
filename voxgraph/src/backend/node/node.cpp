//
// Created by victor on 04.04.19.
//

#include "voxgraph/backend/node/node.h"

namespace voxgraph {
void Node::addToProblem(ceres::Problem *problem) {
  problem->AddParameterBlock(world_node_pose_.data(), world_node_pose_.size());
  if (constant_) {
    problem->SetParameterBlockConstant(world_node_pose_.data());
  }
}
}  // namespace voxgraph
