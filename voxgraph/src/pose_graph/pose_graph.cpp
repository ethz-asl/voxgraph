//
// Created by victor on 17.01.19.
//

#include "voxgraph/pose_graph/pose_graph.h"
#include <map>
#include <memory>
#include <utility>

namespace voxgraph {
void PoseGraph::addNode(const SubmapNode::Config &config) {
  // Add to the node set
  auto ptr = std::make_shared<SubmapNode>(config);
  node_map_.emplace(newNodeId(), std::static_pointer_cast<Node>(ptr));
}

void PoseGraph::addConstraint(const OdometryConstraint::Config &config) {
  // Set the constraint's endpoints
  Node::NodeId first_node_id, second_node_id;
  // TODO(victorr): Find node ids corresponding to endpoints
  Constraint::Endpoints endpoints = {first_node_id, second_node_id};

  // Add to the constraint set
  auto ptr = std::make_shared<OdometryConstraint>(config, endpoints);
  constraints_.emplace_back(std::static_pointer_cast<Constraint>(ptr));
}

void PoseGraph::addConstraint(const RegistrationConstraint::Config &config) {
  CHECK_NE(config.first_submap_id, config.second_submap_id)
      << "Cannot constrain submap " << config.first_submap_id << " to itself";

  // Find the node ids corresponding to the given submap ids
  Node::NodeId first_node_id, second_node_id;
  std::pair<bool, bool> node_ids_found = {false, false};
  for (std::pair<const Node::NodeId &, const Node::Ptr &> node_kv : node_map_) {
    auto submap_node_ptr = dynamic_cast<SubmapNode *>(node_kv.second.get());
    if (submap_node_ptr) {
      cblox::SubmapID submap_id = submap_node_ptr->getSubmapId();
      if (submap_id == config.first_submap_id) {
        first_node_id = node_kv.first;
        node_ids_found.first = true;
      } else if (submap_id == config.second_submap_id) {
        second_node_id = node_kv.first;
        node_ids_found.second = true;
      }
      if (node_ids_found.first && node_ids_found.second) {
        break;
      }
    }
  }
  CHECK(node_ids_found.first) << "Graph contains no node for submap "
                              << config.first_submap_id;
  CHECK(node_ids_found.second) << "Graph contains no node for submap "
                               << config.second_submap_id;

  // Set the constraint's endpoints
  Constraint::Endpoints endpoints = {first_node_id, second_node_id};

  // Get pointers to both submaps
  VoxgraphSubmap::ConstPtr first_submap_ptr =
      submap_collection_ptr_->getSubMapConstPtrById(config.first_submap_id);
  VoxgraphSubmap::ConstPtr second_submap_ptr =
      submap_collection_ptr_->getSubMapConstPtrById(config.second_submap_id);
  CHECK_NOTNULL(first_submap_ptr);
  CHECK_NOTNULL(second_submap_ptr);

  // Add to the constraint set
  auto ptr = std::make_shared<RegistrationConstraint>(
      config, endpoints, first_submap_ptr, second_submap_ptr);
  constraints_.emplace_back(std::static_pointer_cast<Constraint>(ptr));
}

void PoseGraph::optimize() {
  // Initialize the problem and add all constraints
  ceres::Problem::Options problem_options;
  ceres::Problem problem(problem_options);

  for (const Constraint::Ptr &constraint : constraints_) {
    constraint->addToProblem(node_map_, &problem);
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
  for (std::pair<const Node::NodeId &, const Node::Ptr &> node_kv : node_map_) {
    auto submap_node_ptr = dynamic_cast<SubmapNode *>(node_kv.second.get());
    if (submap_node_ptr) {
      submap_poses.emplace(submap_node_ptr->getSubmapId(),
                           submap_node_ptr->getSubmapPose());
    }
  }
  return submap_poses;
}
}  // namespace voxgraph
