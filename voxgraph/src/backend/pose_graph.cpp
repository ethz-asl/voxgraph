#include "voxgraph/backend/pose_graph.h"

#include <map>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "voxgraph/backend/node/pose/pose.h"

namespace voxgraph {
PoseGraph::PoseGraph(std::string pose_graph_name)
    : pose_graph_name_(std::move(pose_graph_name)) {
  // Set problem options
  problem_options_.local_parameterization_ownership =
      ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;
}

void PoseGraph::setConfig(const Config& config) {
  // Set solver options
  solver_options_.num_threads = config.num_threads;
  solver_options_.parameter_tolerance = config.parameter_tolerance;
  solver_options_.max_solver_time_in_seconds =
      config.max_solver_time_in_seconds;
  solver_options_.max_num_iterations = config.max_num_iterations;
  switch (config.solver_type) {
    case Config::SolverType::kDenseSchur:
      solver_options_.linear_solver_type = ceres::LinearSolverType::DENSE_SCHUR;
      break;
    case Config::SolverType::kSparseSchur:
    default:
      solver_options_.linear_solver_type =
          ceres::LinearSolverType::SPARSE_SCHUR;
      break;
  }
}

void PoseGraph::addSubmapNode(const SubmapNode::Config& config) {
  node_collection_.addSubmapNode(config);
}

bool PoseGraph::hasSubmapNode(const voxgraph::SubmapNode::SubmapId& submap_id) {
  auto ptr = node_collection_.getSubmapNodePtrById(submap_id);
  return ptr != nullptr;
}

bool PoseGraph::setSubmapNodeConstant(const SubmapNode::SubmapId& submap_id,
                                      const bool constant) {
  SubmapNode::Ptr submap_node_ptr =
      node_collection_.getSubmapNodePtrById(submap_id);
  if (submap_node_ptr) {
    submap_node_ptr->setConstant(constant);
    return true;
  }
  return false;
}

PoseGraph::SubmapNodeConstnessMap PoseGraph::getSubmapNodeConstness() {
  SubmapNodeConstnessMap submap_constness_map;
  for (const auto& submap_node : node_collection_.getSubmapNodes()) {
    submap_constness_map[submap_node.first] = submap_node.second->isConstant();
  }
  return submap_constness_map;
}

void PoseGraph::addReferenceFrameNode(
    const ReferenceFrameNode::Config& config) {
  node_collection_.addReferenceFrameNode(config);
}

bool PoseGraph::hasReferenceFrameNode(
    const ReferenceFrameNode::FrameId& frame_id) {
  auto ptr = node_collection_.getReferenceFrameNodePtrById(frame_id);
  return ptr != nullptr;
}

bool PoseGraph::setReferenceFramePose(
    const ReferenceFrameNode::FrameId& frame_id,
    const Transformation& reference_frame_pose) {
  ReferenceFrameNode::Ptr reference_frame_node_ptr =
      node_collection_.getReferenceFrameNodePtrById(frame_id);
  if (reference_frame_node_ptr) {
    *reference_frame_node_ptr->getPosePtr() = reference_frame_pose;
    return true;
  }
  return false;
}

void PoseGraph::addAbsolutePoseConstraint(
    const voxgraph::AbsolutePoseConstraint::Config& config) {
  // Add to the constraint set
  constraints_collection_.addAbsolutePoseConstraint(config);
}

void PoseGraph::addRelativePoseConstraint(
    const RelativePoseConstraint::Config& config) {
  // Add to the constraint set
  constraints_collection_.addRelativePoseConstraint(config);
}

void PoseGraph::addRegistrationConstraint(
    const RegistrationConstraint::Config& config) {
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

void PoseGraph::optimize() {
  // Initialize the problem
  problem_ptr_ = std::make_shared<ceres::Problem>(problem_options_);

  // Add the appropriate constraints
  constraints_collection_.addConstraintsToProblem(node_collection_,
                                                  problem_ptr_.get());

  // Run the solver
  ceres::Solver::Summary summary;
  ceres::Solve(solver_options_, problem_ptr_.get(), &summary);

  // Display and store the solver summary
  std::cout << "Optimized " << pose_graph_name_ << ":\n"
            << "- parameter blocks original " << summary.num_parameter_blocks
            << " -> reduced " << summary.num_parameter_blocks_reduced << "\n"
            << "- cost initial " << summary.initial_cost << " -> final "
            << summary.final_cost << "\n"
            << "- iterations successful " << summary.num_successful_steps
            << " -> unsuccessful " << summary.num_unsuccessful_steps << "\n"
            << "- total time " << summary.total_time_in_seconds << "\n"
            << "- termination "
            << ceres::TerminationTypeToString(summary.termination_type)
            << std::endl;
  solver_summaries_.emplace_back(summary);

  // TODO(victorr): Make this check more formal
  CHECK(summary.IsSolutionUsable());
}

bool PoseGraph::getSubmapPose(const SubmapID submap_id,
                              Transformation* submap_pose) {
  CHECK_NOTNULL(submap_pose);
  SubmapNode::Ptr submap_node_ptr =
      node_collection_.getSubmapNodePtrById(submap_id);
  if (submap_node_ptr) {
    *submap_pose = submap_node_ptr->getPose();
    return true;
  } else {
    return false;
  }
}

PoseGraph::PoseMap PoseGraph::getSubmapPoses() {
  PoseMap submap_poses;
  for (const auto& submap_node_kv : node_collection_.getSubmapNodes()) {
    submap_poses.emplace(submap_node_kv.second->getSubmapId(),
                         submap_node_kv.second->getPose());
  }
  return submap_poses;
}

bool PoseGraph::setSubmapPose(const SubmapID submap_id,
                              const Transformation& submap_pose) {
  SubmapNode::Ptr submap_node_ptr =
      node_collection_.getSubmapNodePtrById(submap_id);
  if (submap_node_ptr) {
    *submap_node_ptr->getPosePtr() = submap_pose;
    return true;
  }
  return false;
}

bool PoseGraph::getEdgeCovarianceMap(
    PoseGraph::EdgeCovarianceMap* edge_covariance_map) const {
  CHECK_NOTNULL(edge_covariance_map);

  // Configure the covariance extraction
  ceres::Covariance::Options options;
  ceres::Covariance covariance(options);
  std::vector<std::pair<const double*, const double*>> covariance_blocks;

  // Request covariance estimates for the submap pairs specified through the
  // edge_covariance_map
  for (std::pair<const SubmapIdPair, EdgeCovarianceMatrix>&
           edge_covariance_pair : *edge_covariance_map) {
    SubmapNode::Ptr first_submap_node =
        node_collection_.getSubmapNodePtrById(edge_covariance_pair.first.first);
    SubmapNode::Ptr second_submap_node = node_collection_.getSubmapNodePtrById(
        edge_covariance_pair.first.second);
    covariance_blocks.emplace_back(
        first_submap_node->getPosePtr()->optimizationVectorData(),
        second_submap_node->getPosePtr()->optimizationVectorData());
  }

  // Compute the requested covariances
  if (!covariance.Compute(covariance_blocks, problem_ptr_.get())) {
    // The covariance computation failed
    return false;
  }

  // Return the estimated covariances by storing them in the edge_covariance_map
  for (std::pair<const SubmapIdPair, EdgeCovarianceMatrix>&
           edge_covariance_pair : *edge_covariance_map) {
    SubmapNode::Ptr first_submap_node =
        node_collection_.getSubmapNodePtrById(edge_covariance_pair.first.first);
    SubmapNode::Ptr second_submap_node = node_collection_.getSubmapNodePtrById(
        edge_covariance_pair.first.second);
    if (!covariance.GetCovarianceBlock(
            first_submap_node->getPosePtr()->optimizationVectorData(),
            second_submap_node->getPosePtr()->optimizationVectorData(),
            edge_covariance_pair.second.data())) {
      // The covariance pair for this submap is missing, which shouldn't happen
      // since it has been requested.
      return false;
    }
  }

  return true;
}

PoseGraph::VisualizationEdgeList PoseGraph::getVisualizationEdges() const {
  // Check if the problem has been initialized
  CHECK_NOTNULL(problem_ptr_);

  // Get the residual blocks and residual values
  std::vector<ceres::ResidualBlockId> residual_block_ids;
  problem_ptr_->GetResidualBlocks(&residual_block_ids);
  std::vector<double> residuals;
  problem_ptr_->Evaluate(ceres::Problem::EvaluateOptions(), nullptr, &residuals,
                         nullptr, nullptr);

  // Iterate over all residual blocks and setup the corresponding edges
  VisualizationEdgeList edges;
  size_t residual_idx = 0;
  for (const ceres::ResidualBlockId& residual_block_id : residual_block_ids) {
    VisualizationEdge edge;

    // Find and store the edge endpoints
    std::vector<double*> edge_endpoints;
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
    const ceres::CostFunction* cost_function_ptr =
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
