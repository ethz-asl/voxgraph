#ifndef VOXGRAPH_BACKEND_POSE_GRAPH_H_
#define VOXGRAPH_BACKEND_POSE_GRAPH_H_

#include <list>
#include <map>
#include <memory>
#include <utility>
#include <vector>
#include "voxgraph/backend/constraint/constraint_collection.h"
#include "voxgraph/backend/node/node_collection.h"
#include "voxgraph/common.h"

namespace voxgraph {
class PoseGraph {
 public:
  typedef std::shared_ptr<const PoseGraph> ConstPtr;
  typedef std::list<ceres::Solver::Summary> SolverSummaryList;
  typedef std::map<const SubmapID, const Transformation> PoseMap;

  PoseGraph() : need_two_stage_opt_(false) {}

  void addSubmapNode(const SubmapNode::Config &config);
  bool hasSubmapNode(const SubmapNode::SubmapId &submap_id);
  void addReferenceFrameNode(const ReferenceFrameNode::Config &config);
  bool hasReferenceFrameNode(const ReferenceFrameNode::FrameId &frame_id);

  void addAbsolutePoseConstraint(const AbsolutePoseConstraint::Config &config);
  void addRelativePoseConstraint(const RelativePoseConstraint::Config &config);
  void addRegistrationConstraint(const RegistrationConstraint::Config &config);

  void resetRegistrationConstraints() {
    constraints_collection_.resetRegistrationConstraints();
  }

  void needsTwoStageOptimization() { need_two_stage_opt_ = true; };

  void initialize(bool non_registration_only = false);
  void optimize();
  void solve();

  PoseMap getSubmapPoses();

  struct VisualizationEdge {
    Transformation::Position first_node_position;
    Transformation::Position second_node_position;
    double residual;
  };
  std::vector<VisualizationEdge> getVisualizationEdges() const;

  const SolverSummaryList &getSolverSummaries() { return solver_summaries_; }

 private:
  ConstraintCollection constraints_collection_;
  NodeCollection node_collection_;

  // Ceres problem
  ceres::Problem::Options problem_options_;
  std::shared_ptr<ceres::Problem> problem_ptr_;
  SolverSummaryList solver_summaries_;

  // Flag indicating that the next optimization should be two staged
  bool need_two_stage_opt_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_BACKEND_POSE_GRAPH_H_
