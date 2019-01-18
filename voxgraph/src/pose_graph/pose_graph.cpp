//
// Created by victor on 17.01.19.
//

#include "voxgraph/pose_graph/pose_graph.h"
#include <map>
#include <memory>
#include <utility>

namespace voxgraph {
// TODO(victorr): Once multiple node types are used, template this method
void PoseGraph::addNode(const SubmapNode::Config &config) {
  // Add to the node set
  node_collection_.addNode(config);
}

void PoseGraph::addConstraint(const OdometryConstraint::Config &config) {
  // TODO(victorr): Implement this
  // Add to the constraint set
  auto ptr = std::make_shared<OdometryConstraint>(newConstraintId(), config);
  constraints_.emplace_back(std::static_pointer_cast<Constraint>(ptr));
}

void PoseGraph::addConstraint(const RegistrationConstraint::Config &config) {
  CHECK_NE(config.first_submap_id, config.second_submap_id)
      << "Cannot constrain submap " << config.first_submap_id << " to itself";

  // Check if there're submap nodes corresponding to both submap IDs
  CHECK(node_collection_.getNodePtrById(config.first_submap_id))
      << "Graph contains no node for submap " << config.first_submap_id;
  CHECK(node_collection_.getNodePtrById(config.second_submap_id))
      << "Graph contains no node for submap " << config.second_submap_id;

  // Get pointers to both submaps
  VoxgraphSubmap::ConstPtr first_submap_ptr =
      submap_collection_ptr_->getSubMapConstPtrById(config.first_submap_id);
  VoxgraphSubmap::ConstPtr second_submap_ptr =
      submap_collection_ptr_->getSubMapConstPtrById(config.second_submap_id);
  CHECK_NOTNULL(first_submap_ptr);
  CHECK_NOTNULL(second_submap_ptr);

  // Add to the constraint set
  auto ptr = std::make_shared<RegistrationConstraint>(
      newConstraintId(), config, first_submap_ptr, second_submap_ptr);
  constraints_.emplace_back(std::static_pointer_cast<Constraint>(ptr));
}

void PoseGraph::optimize() {
  // Initialize the problem and add all constraints
  ceres::Problem::Options problem_options;
  ceres::Problem problem(problem_options);

  for (const Constraint::Ptr &constraint : constraints_) {
    constraint->addToProblem(node_collection_, &problem);
  }

  // Run the solver
  ceres::Solver::Options ceres_options;
  ceres::Solver::Summary summary;
  ceres::Solve(ceres_options, &problem, &summary);

  std::cout << summary.FullReport() << std::endl;
}

std::map<const cblox::SubmapID, const voxblox::Transformation>
PoseGraph::getSubmapPoses() {
  std::map<const cblox::SubmapID, const voxblox::Transformation> submap_poses;
  for (auto submap_node_kv : node_collection_.getSubmapNodes()) {
    submap_poses.emplace(submap_node_kv.second->getSubmapId(),
                         submap_node_kv.second->getSubmapPose());
  }
  return submap_poses;
}
}  // namespace voxgraph
