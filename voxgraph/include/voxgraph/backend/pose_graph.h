//
// Created by victor on 15.01.19.
//

#ifndef VOXGRAPH_BACKEND_POSE_GRAPH_H_
#define VOXGRAPH_BACKEND_POSE_GRAPH_H_

#include <map>
#include <memory>
#include <utility>
#include <vector>
#include "voxgraph/backend/constraint/absolute_pose_constraint.h"
#include "voxgraph/backend/constraint/loop_closure_constraint.h"
#include "voxgraph/backend/constraint/odometry_constraint.h"
#include "voxgraph/backend/constraint/registration_constraint.h"
#include "voxgraph/backend/node/submap_node.h"

namespace voxgraph {
class PoseGraph {
 public:
  typedef std::shared_ptr<const PoseGraph> ConstPtr;

  explicit PoseGraph(
      cblox::SubmapCollection<VoxgraphSubmap>::ConstPtr submap_collection_ptr)
      : submap_collection_ptr_(std::move(submap_collection_ptr)) {}

  void addSubmapNode(const SubmapNode::Config &config);

  void addRegistrationConstraint(const RegistrationConstraint::Config &config);
  void addOdometryConstraint(const OdometryConstraint::Config &config);
  void addLoopClosureConstraint(const LoopClosureConstraint::Config &config);
  void addAbsolutePoseConstraint(const AbsolutePoseConstraint::Config &config);

  void initialize();

  void optimize();

  std::map<const cblox::SubmapID, const voxblox::Transformation>
  getSubmapPoses();

  struct Edge {
    voxblox::Transformation::Position first_node_position;
    voxblox::Transformation::Position second_node_position;
    double residual;
  };
  std::vector<Edge> getEdges() const;

 private:
  Constraint::ConstraintId constraint_id_counter_ = 0;
  const Constraint::ConstraintId newConstraintId() {
    return constraint_id_counter_++;
  }
  std::vector<Constraint::Ptr> constraints_;

  NodeCollection node_collection_;
  cblox::SubmapCollection<VoxgraphSubmap>::ConstPtr submap_collection_ptr_;

  // Ceres problem
  ceres::Problem::Options problem_options_;
  std::shared_ptr<ceres::Problem> problem_ptr_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_BACKEND_POSE_GRAPH_H_
