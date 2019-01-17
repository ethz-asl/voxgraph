//
// Created by victor on 15.01.19.
//

#ifndef VOXGRAPH_POSE_GRAPH_NODE_H_
#define VOXGRAPH_POSE_GRAPH_NODE_H_

#include <map>
#include <memory>

namespace voxgraph {
class Node {
 public:
  typedef std::shared_ptr<Node> Ptr;
  typedef unsigned int NodeId;
  typedef std::map<Node::NodeId, Node::Ptr> NodeMap;
  typedef std::array<double, 3> Pose;

  Node() = default;
  virtual ~Node() = default;

  Pose* getPosePtr() { return &world_t_world__node_pose_; }

 protected:
  Pose world_t_world__node_pose_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_POSE_GRAPH_NODE_H_
