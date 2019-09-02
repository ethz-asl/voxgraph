#include "voxgraph/backend/pose_graph.h"
#include <map>
#include <memory>
#include <thread>
#include <utility>
#include <vector>
#include "voxgraph/backend/node/pose/pose.h"

namespace voxgraph {
void PoseGraph::addSubmapNode(const SubmapNode::Config &config) {
  node_collection_.addSubmapNode(config);
}

bool PoseGraph::hasSubmapNode(const voxgraph::SubmapNode::SubmapId &submap_id) {
  auto ptr = node_collection_.getSubmapNodePtrById(submap_id);
  return ptr != nullptr;
}

void PoseGraph::addReferenceFrameNode(
    const ReferenceFrameNode::Config &config) {
  node_collection_.addReferenceFrameNode(config);
}

bool PoseGraph::hasReferenceFrameNode(
    const ReferenceFrameNode::FrameId &frame_id) {
  auto ptr = node_collection_.getReferenceFrameNodePtrById(frame_id);
  return ptr != nullptr;
}

void PoseGraph::addAbsolutePoseConstraint(
    const voxgraph::AbsolutePoseConstraint::Config &config) {
  // TODO(victorr): Add check on whether both endpoints exist

  // Add to the constraint set
  constraints_collection_.addAbsolutePoseConstraint(config);
}

void PoseGraph::addRelativePoseConstraint(
    const RelativePoseConstraint::Config &config) {
  // TODO(victorr): Add check on whether both endpoints exist

  // Add to the constraint set
  constraints_collection_.addRelativePoseConstraint(config);
}

void PoseGraph::addRegistrationConstraint(
    const RegistrationConstraint::Config &config) {
  CHECK_NE(config.first_submap_id, config.second_submap_id)
      << "Cannot constrain submap " << config.first_submap_id << " to itself";

  // Check if there're submap nodes corresponding to both submap IDs
  CHECK(node_collection_.getSubmapNodePtrById(config.first_submap_id))
      << "Graph contains no node for submap " << config.first_submap_id;
  CHECK(node_collection_.getSubmapNodePtrById(config.second_submap_id))
      << "Graph contains no node for submap " << config.second_submap_id;

  // Add to the constraint set
  constraints_collection_.addRegistrationConstraint(config);
}

void PoseGraph::initialize(bool non_registration_only) {
  // Initialize the problem and add all constraints
  problem_options_.local_parameterization_ownership =
      ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;
  problem_ptr_.reset(new ceres::Problem(problem_options_));
  if (non_registration_only) {
    constraints_collection_.addNonRegistrationToProblem(node_collection_,
                                                        problem_ptr_.get());
  } else {
    constraints_collection_.addAllToProblem(node_collection_, problem_ptr_.get());
  }
}

void PoseGraph::optimize() {
  // The prestage solution optimizes the graph without regiration constraints to
  // try to get out of local minima
  if (need_two_stage_opt_) {
    std::cout << "Performing the pre-stage solve." << std::endl;
    static constexpr bool nonRegistrationOnlyFlag = true;
    initialize(nonRegistrationOnlyFlag);
    solve();
    need_two_stage_opt_ = false;
    std::cout << "Now performing the main solve." << std::endl;
  }
  // The main optimization
  initialize();
  solve();
}

void PoseGraph::solve() {
  // Run the solver
  ceres::Solver::Options ceres_options;
  // TODO(victorr): Set these from parameters
  // TODO(victorr): Look into manual parameter block ordering
  ceres_options.parameter_tolerance = 3e-4;
  //  ceres_options.max_num_iterations = 4;
  //  ceres_options.max_solver_time_in_seconds = 12;
  ceres_options.num_threads = std::thread::hardware_concurrency();
  ceres_options.linear_solver_type = ceres::LinearSolverType::DENSE_SCHUR;
  // ceres_options.linear_solver_type = ceres::LinearSolverType::SPARSE_SCHUR;
  // NOTE: For small problems DENSE_SCHUR is much faster

  ceres::Solver::Summary summary;
  ceres::Solve(ceres_options, problem_ptr_.get(), &summary);

  // Display and store the solver summary
  std::cout << summary.FullReport() << std::endl;
  solver_summaries_.emplace_back(summary);
}

PoseGraph::PoseMap PoseGraph::getSubmapPoses() {
  PoseMap submap_poses;
  for (const auto &submap_node_kv : node_collection_.getSubmapNodes()) {
    submap_poses.emplace(submap_node_kv.second->getSubmapId(),
                         submap_node_kv.second->getPose());
  }
  return submap_poses;
}

std::vector<PoseGraph::VisualizationEdge> PoseGraph::getVisualizationEdges()
    const {
  // Check if the problem has been initialized
  CHECK_NOTNULL(problem_ptr_);

  // Get the residual blocks and residual values
  std::vector<ceres::ResidualBlockId> residual_block_ids;
  problem_ptr_->GetResidualBlocks(&residual_block_ids);
  std::vector<double> residuals;
  problem_ptr_->Evaluate(ceres::Problem::EvaluateOptions(), nullptr, &residuals,
                         nullptr, nullptr);

  // Iterate over all residual blocks and setup the corresponding edges
  std::vector<VisualizationEdge> edges;
  size_t residual_idx = 0;
  for (const ceres::ResidualBlockId &residual_block_id : residual_block_ids) {
    VisualizationEdge edge;

    // Find and store the edge endpoints
    std::vector<double *> edge_endpoints;
    problem_ptr_->GetParameterBlocksForResidualBlock(residual_block_id,
                                                     &edge_endpoints);
    CHECK_EQ(edge_endpoints.size(), 2);
    edge.first_node_position.x() = edge_endpoints[0][0];
    edge.first_node_position.y() = edge_endpoints[0][1];
    edge.first_node_position.z() = edge_endpoints[0][2];
    edge.second_node_position.x() = edge_endpoints[1][0];
    edge.second_node_position.y() = edge_endpoints[1][1];
    edge.second_node_position.z() = edge_endpoints[1][2];

    // Calculate the total of all cost residuals on the current edge
    // NOTE: We visualize one edge per submap-submap constraint, however
    //       there is one residual per voxel-voxel correspondence.
    //       This is what the summation below is for.
    edge.residual = 0;
    const ceres::CostFunction *cost_function_ptr =
        problem_ptr_->GetCostFunctionForResidualBlock(residual_block_id);
    CHECK_NOTNULL(cost_function_ptr);
    int num_residuals_for_edge = cost_function_ptr->num_residuals();
    for (int i = 0; i < num_residuals_for_edge; i++) {
      edge.residual += residuals[residual_idx] * residuals[residual_idx];
      residual_idx++;
    }
    edges.emplace_back(edge);
  }

  return edges;
}
}  // namespace voxgraph
