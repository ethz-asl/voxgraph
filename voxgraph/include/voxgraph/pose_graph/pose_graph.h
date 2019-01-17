//
// Created by victor on 15.01.19.
//

#ifndef VOXGRAPH_POSE_GRAPH_POSE_GRAPH_H_
#define VOXGRAPH_POSE_GRAPH_POSE_GRAPH_H_

#include <map>
#include <utility>
#include <vector>
#include "voxgraph/pose_graph/odometry_constraint.h"
#include "voxgraph/pose_graph/registration_constraint.h"
#include "voxgraph/pose_graph/submap_node.h"

namespace voxgraph {
class PoseGraph {
 public:
  explicit PoseGraph(
      cblox::SubmapCollection<VoxgraphSubmap>::ConstPtr submap_collection_ptr)
      : submap_collection_ptr_(std::move(submap_collection_ptr)) {}

  void addNode(const SubmapNode::Config &config);

  void addConstraint(const OdometryConstraint::Config &config);

  void addConstraint(const RegistrationConstraint::Config &config);

  void optimize();

  std::map<const cblox::SubmapID, const voxblox::Transformation>
  getSubmapPoses();

 private:
  Node::NodeId node_id_counter_ = 0;
  Node::NodeId newNodeId() { return node_id_counter_++; }

  Node::NodeMap node_map_;
  std::vector<Constraint::Ptr> constraints_;
  cblox::SubmapCollection<VoxgraphSubmap>::ConstPtr submap_collection_ptr_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_POSE_GRAPH_POSE_GRAPH_H_
