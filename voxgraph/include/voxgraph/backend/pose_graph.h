//
// Created by victor on 15.01.19.
//

#ifndef VOXGRAPH_BACKEND_POSE_GRAPH_H_
#define VOXGRAPH_BACKEND_POSE_GRAPH_H_

#include <map>
#include <memory>
#include <utility>
#include <vector>
#include "voxgraph/backend/constraint/constraint_collection.h"
#include "voxgraph/backend/node/node_collection.h"

namespace voxgraph {
class PoseGraph {
 public:
  typedef std::shared_ptr<const PoseGraph> ConstPtr;

  PoseGraph() = default;

  void addSubmapNode(const SubmapNode::Config &config);

  void addAbsolutePoseConstraint(const AbsolutePoseConstraint::Config &config);
  void addLoopClosureConstraint(const LoopClosureConstraint::Config &config);
  void addOdometryConstraint(const OdometryConstraint::Config &config);
  void addRegistrationConstraint(const RegistrationConstraint::Config &config);

  void resetRegistrationConstraints() {
    constraints_collection_.resetRegistrationConstraints();
  }

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
  ConstraintCollection constraints_collection_;
  NodeCollection node_collection_;

  // Ceres problem
  ceres::Problem::Options problem_options_;
  std::shared_ptr<ceres::Problem> problem_ptr_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_BACKEND_POSE_GRAPH_H_
