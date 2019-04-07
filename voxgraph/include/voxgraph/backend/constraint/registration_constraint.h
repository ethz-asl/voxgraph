//
// Created by victor on 16.01.19.
//

#ifndef VOXGRAPH_BACKEND_CONSTRAINT_REGISTRATION_CONSTRAINT_H_
#define VOXGRAPH_BACKEND_CONSTRAINT_REGISTRATION_CONSTRAINT_H_

#include <memory>
#include <utility>
#include "voxgraph/backend/constraint/constraint.h"
#include "voxgraph/backend/constraint/cost_functions/submap_registration/registration_cost_function_xyz_yaw.h"

namespace voxgraph {
class RegistrationConstraint : public Constraint {
 public:
  typedef std::shared_ptr<RegistrationConstraint> Ptr;
  struct Config : Constraint::Config {
    cblox::SubmapID first_submap_id;
    cblox::SubmapID second_submap_id;
  };

  explicit RegistrationConstraint(ConstraintId constraint_id,
                                  const Config& config,
                                  VoxgraphSubmap::ConstPtr first_submap_ptr,
                                  VoxgraphSubmap::ConstPtr second_submap_ptr)
      : Constraint(constraint_id, config),
        config_(config),
        first_submap_ptr_(std::move(first_submap_ptr)),
        second_submap_ptr_(std::move(second_submap_ptr)) {}

  // TODO(victorr): Use the sqrt_information_matrix_ when computing the residual
  void addToProblem(const NodeCollection& node_collection,
                    ceres::Problem* problem) final;

 private:
  Config config_;
  VoxgraphSubmap::ConstPtr first_submap_ptr_;
  VoxgraphSubmap::ConstPtr second_submap_ptr_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_BACKEND_CONSTRAINT_REGISTRATION_CONSTRAINT_H_
