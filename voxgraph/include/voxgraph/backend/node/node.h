//
// Created by victor on 15.01.19.
//

#ifndef VOXGRAPH_BACKEND_NODE_NODE_H_
#define VOXGRAPH_BACKEND_NODE_NODE_H_

#include <ceres/ceres.h>
#include <voxblox/core/common.h>
#include <memory>

namespace voxgraph {
class Node {
 public:
  typedef std::shared_ptr<Node> Ptr;
  typedef unsigned int NodeId;
  typedef std::array<double, 4> Pose;

  struct Config {
    bool set_constant;
    voxblox::Transformation T_world_node_initial;
  };

  explicit Node(const NodeId &node_id, const Config &config);
  virtual ~Node() = default;

  const voxblox::Transformation getPose() const;
  Pose *getPosePtr() { return &optimized_pose_; }

  void setConstant(bool constant) { config_.set_constant = constant; }
  bool isConstant() { return config_.set_constant; }

  void addToProblem(ceres::Problem *problem,
                    ceres::LocalParameterization *local_parameterization);

 protected:
  const NodeId node_id_;
  Config config_;

  Pose optimized_pose_{};
};
}  // namespace voxgraph

#endif  // VOXGRAPH_BACKEND_NODE_NODE_H_
