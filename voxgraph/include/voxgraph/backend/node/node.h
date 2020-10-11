#ifndef VOXGRAPH_BACKEND_NODE_NODE_H_
#define VOXGRAPH_BACKEND_NODE_NODE_H_

#include <memory>

#include <ceres/ceres.h>
#include <voxblox/core/common.h>
#include <voxgraph/backend/node/pose/pose_4d.h>

namespace voxgraph {
class Node {
 public:
  typedef std::shared_ptr<Node> Ptr;
  typedef unsigned int NodeId;

  struct Config {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    bool set_constant;
    // Initial pose of the node in the inertial (non-robocentric) frame
    voxblox::Transformation T_I_node_initial;
  };

  Node(const Node::NodeId& node_id, const Config& config)
      : node_id_(node_id),
        config_(config),
        optimized_pose_(config.T_I_node_initial) {}
  virtual ~Node() = default;

  const Pose& getPose() const { return optimized_pose_; }
  Pose* getPosePtr() { return &optimized_pose_; }

  void setConstant(bool constant) { config_.set_constant = constant; }
  bool isConstant() { return config_.set_constant; }

  void addToProblem(ceres::Problem* problem,
                    ceres::LocalParameterization* local_parameterization);

 protected:
  const NodeId node_id_;
  Config config_;

  Pose4D optimized_pose_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_BACKEND_NODE_NODE_H_
