//
// Created by victor on 16.01.19.
//

#ifndef VOXGRAPH_POSE_GRAPH_CONSTRAINT_REGISTRATION_CONSTRAINT_H_
#define VOXGRAPH_POSE_GRAPH_CONSTRAINT_REGISTRATION_CONSTRAINT_H_

#include <memory>
#include <utility>
#include "voxgraph/pose_graph/constraint/constraint.h"
#include "voxgraph/submap_registration/registration_cost_function.h"

namespace voxgraph {
class RegistrationConstraint : public Constraint {
 public:
  typedef std::shared_ptr<RegistrationConstraint> Ptr;
  struct Config {
    cblox::SubmapID first_submap_id;
    cblox::SubmapID second_submap_id;
  };

  explicit RegistrationConstraint(ConstraintId constraint_id, Config config,
                                  VoxgraphSubmap::ConstPtr first_submap_ptr,
                                  VoxgraphSubmap::ConstPtr second_submap_ptr)
      : Constraint(constraint_id),
        config_(config),
        first_submap_ptr_(std::move(first_submap_ptr)),
        second_submap_ptr_(std::move(second_submap_ptr)) {}

  void addToProblem(const NodeCollection& node_collection,
                    ceres::Problem* problem) final;

 private:
  Config config_;
  VoxgraphSubmap::ConstPtr first_submap_ptr_;
  VoxgraphSubmap::ConstPtr second_submap_ptr_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_POSE_GRAPH_CONSTRAINT_REGISTRATION_CONSTRAINT_H_
