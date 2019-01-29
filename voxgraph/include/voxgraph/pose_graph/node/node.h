//
// Created by victor on 15.01.19.
//

#ifndef VOXGRAPH_POSE_GRAPH_NODE_NODE_H_
#define VOXGRAPH_POSE_GRAPH_NODE_NODE_H_

#include <memory>

namespace voxgraph {
class Node {
 public:
  typedef std::shared_ptr<Node> Ptr;
  typedef unsigned int NodeId;
  typedef std::array<double, 4> Pose;

  struct Config {
    bool set_constant;
  };

  explicit Node(NodeId node_id) : node_id_(node_id) {}
  virtual ~Node() = default;

  Pose* getPosePtr() { return &world_node_pose_; }
  void setConstant(bool constant) { constant_ = constant; }
  bool isConstant() { return constant_; }

 protected:
  const NodeId node_id_;
  Pose world_node_pose_;
  bool constant_ = false;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_POSE_GRAPH_NODE_NODE_H_
