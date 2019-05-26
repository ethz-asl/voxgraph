#include "voxgraph/backend/node/node.h"

namespace voxgraph {
Node::Node(const Node::NodeId &node_id, const Config &config)
    : node_id_(node_id), config_(config) {
  // Set the node's optimization pose to the initial pose
  voxblox::Transformation::Vector6 T_vec = config_.T_world_node_initial.log();
  optimized_pose_[0] = T_vec[0];
  optimized_pose_[1] = T_vec[1];
  optimized_pose_[2] = T_vec[2];
  optimized_pose_[3] = T_vec[5];
}

void Node::addToProblem(ceres::Problem *problem,
                        ceres::LocalParameterization *local_parameterization) {
  CHECK_NOTNULL(problem);
  CHECK_NOTNULL(local_parameterization);

  // Add the node to the solver and set it to be held constant if appropriate
  problem->AddParameterBlock(optimized_pose_.data(), optimized_pose_.size());
  if (config_.set_constant) {
    problem->SetParameterBlockConstant(optimized_pose_.data());
  }

  // Set the local parameterization s.t. yaw stays normalized
  problem->SetParameterization(getPosePtr()->data(), local_parameterization);
}

const voxblox::Transformation Node::getPose() const {
  voxblox::Transformation::Vector6 T_vec = config_.T_world_node_initial.log();
  T_vec[0] = optimized_pose_[0];
  T_vec[1] = optimized_pose_[1];
  T_vec[2] = optimized_pose_[2];
  T_vec[5] = optimized_pose_[3];
  return voxblox::Transformation::exp(T_vec);
}
}  // namespace voxgraph
