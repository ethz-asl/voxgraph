#ifndef VOXGRAPH_BACKEND_POSE_GRAPH_H_
#define VOXGRAPH_BACKEND_POSE_GRAPH_H_

#include <list>
#include <map>
#include <memory>
#include <string>
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
  typedef std::map<SubmapID, bool> SubmapNodeConstnessMap;

  struct Config {
    int num_threads = 2;
    double parameter_tolerance = 3e-3;
    double max_solver_time_in_seconds = 60.0;
    int max_num_iterations = 20;
    enum class SolverType {
      kSparseSchur,
      kDenseSchur
    } solver_type = SolverType::kSparseSchur;
  };

  explicit PoseGraph(std::string pose_graph_name = "");
  // TODO(victorr): Switch to config_utilities
  void setConfig(const Config& config);

  // Submap node getters and setters
  void addSubmapNode(const SubmapNode::Config& config);
  bool hasSubmapNode(const SubmapNode::SubmapId& submap_id);
  bool setSubmapNodeConstant(const SubmapNode::SubmapId& submap_id,
                             const bool constant);
  const NodeCollection::SubmapNodeMap& getSubmapNodes() {
    return node_collection_.getSubmapNodes();
  }
  SubmapNodeConstnessMap getSubmapNodeConstness();
  bool getSubmapPose(const SubmapID submap_id, Transformation* submap_pose);
  PoseMap getSubmapPoses();
  bool setSubmapPose(const SubmapID submap_id,
                     const Transformation& submap_pose);

  // Reference frame node getters and setters
  void addReferenceFrameNode(const ReferenceFrameNode::Config& config);
  bool hasReferenceFrameNode(const ReferenceFrameNode::FrameId& frame_id);
  const NodeCollection::ReferenceFrameNodeMap& getReferenceFrameNodes() {
    return node_collection_.getReferenceFrameNodes();
  }
  bool setReferenceFramePose(const ReferenceFrameNode::FrameId& frame_id,
                             const Transformation& reference_frame_pose);

  // Absolute pose constraint getters and setters
  void addAbsolutePoseConstraint(const AbsolutePoseConstraint::Config& config);
  const ConstraintCollection::AbsolutePoseConstraintList&
  getAbsolutePoseConstraints() {
    return constraints_collection_.getAbsolutePoseConstraints();
  }
  void resetAbsolutePoseConstraints() {}

  // Relative pose constraint getters and setters
  void addRelativePoseConstraint(const RelativePoseConstraint::Config& config);
  const ConstraintCollection::RelativePoseConstraintList&
  getRelativePoseConstraints() {
    return constraints_collection_.getRelativePoseConstraints();
  }
  void resetRelativePoseConstraints() {}

  // Registration constraint getters and setters
  void addRegistrationConstraint(const RegistrationConstraint::Config& config);
  const ConstraintCollection::RegistrationConstraintList&
  getRegistrationConstraints() {
    return constraints_collection_.getRegistrationConstraints();
  }
  void resetRegistrationConstraints() {
    constraints_collection_.resetRegistrationConstraints();
  }

  void optimize();

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
  std::string pose_graph_name_;

  // Pose graph nodes and edges
  NodeCollection node_collection_;
  ConstraintCollection constraints_collection_;

  // Ceres options, problem and stats
  ceres::Solver::Options solver_options_;
  ceres::Problem::Options problem_options_;
  std::shared_ptr<ceres::Problem> problem_ptr_;
  SolverSummaryList solver_summaries_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_BACKEND_POSE_GRAPH_H_
