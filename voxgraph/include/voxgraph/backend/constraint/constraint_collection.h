//
// Created by victor on 09.04.19.
//

#ifndef VOXGRAPH_BACKEND_CONSTRAINT_CONSTRAINT_COLLECTION_H_
#define VOXGRAPH_BACKEND_CONSTRAINT_CONSTRAINT_COLLECTION_H_

#include <list>
#include "voxgraph/backend/constraint/absolute_pose_constraint.h"
#include "voxgraph/backend/constraint/loop_closure_constraint.h"
#include "voxgraph/backend/constraint/odometry_constraint.h"
#include "voxgraph/backend/constraint/registration_constraint.h"

namespace voxgraph {
class ConstraintCollection {
 public:
  void addAbsolutePoseConstraint(const AbsolutePoseConstraint::Config &config) {
    absolute_pose_constraints_.emplace_back(newConstraintId(), config);
  }
  void addLoopClosureConstraint(const LoopClosureConstraint::Config &config) {
    loop_closure_constraints_.emplace_back(newConstraintId(), config);
  }
  void addOdometryConstraint(const OdometryConstraint::Config &config) {
    odometry_constraints_.emplace_back(newConstraintId(), config);
  }
  void addRegistrationConstraint(const RegistrationConstraint::Config &config) {
    registration_constraints_.emplace_back(newConstraintId(), config);
  }

  void resetRegistrationConstraints() { registration_constraints_.clear(); }

  void addAllToProblem(const NodeCollection &node_collection,
                       ceres::Problem *problem_ptr);

 private:
  Constraint::ConstraintId constraint_id_counter_ = 0;
  const Constraint::ConstraintId newConstraintId() {
    return constraint_id_counter_++;
  }

  std::list<AbsolutePoseConstraint> absolute_pose_constraints_;
  std::list<LoopClosureConstraint> loop_closure_constraints_;
  std::list<OdometryConstraint> odometry_constraints_;
  std::list<RegistrationConstraint> registration_constraints_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_BACKEND_CONSTRAINT_CONSTRAINT_COLLECTION_H_
