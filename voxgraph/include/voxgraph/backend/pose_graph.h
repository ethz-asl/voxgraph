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

  PoseGraph() = default;

  void addSubmapNode(const SubmapNode::Config& config);
  bool hasSubmapNode(const SubmapNode::SubmapId& submap_id);
  void addReferenceFrameNode(const ReferenceFrameNode::Config& config);
  bool hasReferenceFrameNode(const ReferenceFrameNode::FrameId& frame_id);

  void addAbsolutePoseConstraint(const AbsolutePoseConstraint::Config& config);
  void addRelativePoseConstraint(const RelativePoseConstraint::Config& config);
  void addRegistrationConstraint(const RegistrationConstraint::Config& config);

  void resetRegistrationConstraints() {
    constraints_collection_.resetRegistrationConstraints();
  }

  void initialize(bool exclude_registration_constraints = false);
  void optimize(bool exclude_registration_constraints = false);

  PoseMap getSubmapPoses();

  typedef Eigen::Matrix<double, 4, 4> EdgeCovarianceMatrix;
  typedef std::map<SubmapIdPair, EdgeCovarianceMatrix> EdgeCovarianceMap;
  bool getEdgeCovarianceMap(EdgeCovarianceMap* edge_covariance_map) const;

  struct VisualizationEdge {
    Transformation::Position first_node_position;
    Transformation::Position second_node_position;
    double residual;
  };
  typedef std::vector<VisualizationEdge> VisualizationEdgeList;
  VisualizationEdgeList getVisualizationEdges() const;

  const SolverSummaryList& getSolverSummaries() const {
    return solver_summaries_;
  }

 private:
  ConstraintCollection constraints_collection_;
  NodeCollection node_collection_;

  // Ceres problem
  ceres::Problem::Options problem_options_;
  std::shared_ptr<ceres::Problem> problem_ptr_;
  SolverSummaryList solver_summaries_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_BACKEND_POSE_GRAPH_H_
