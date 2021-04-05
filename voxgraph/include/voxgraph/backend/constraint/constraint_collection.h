#ifndef VOXGRAPH_BACKEND_CONSTRAINT_CONSTRAINT_COLLECTION_H_
#define VOXGRAPH_BACKEND_CONSTRAINT_CONSTRAINT_COLLECTION_H_

#include <list>

#include "voxgraph/backend/constraint/absolute_pose_constraint.h"
#include "voxgraph/backend/constraint/registration_constraint.h"
#include "voxgraph/backend/constraint/relative_pose_constraint.h"

namespace voxgraph {
class ConstraintCollection {
 public:
  void addAbsolutePoseConstraint(const AbsolutePoseConstraint::Config& config) {
    absolute_pose_constraints_.emplace_back(newConstraintId(), config);
  }
  void addRelativePoseConstraint(const RelativePoseConstraint::Config& config) {
    relative_pose_constraints_.emplace_back(newConstraintId(), config);
  }
  void addRegistrationConstraint(const RegistrationConstraint::Config& config) {
    registration_constraints_.emplace_back(newConstraintId(), config);
  }

  void resetRegistrationConstraints() { registration_constraints_.clear(); }

  void addConstraintsToProblem(const NodeCollection& node_collection,
                               ceres::Problem* problem_ptr,
                               bool exclude_registration_constraints = false);

 private:
  Constraint::ConstraintId constraint_id_counter_ = 0;
  Constraint::ConstraintId newConstraintId() {
    return constraint_id_counter_++;
  }

  std::list<AbsolutePoseConstraint> absolute_pose_constraints_;
  std::list<RelativePoseConstraint> relative_pose_constraints_;
  std::list<RegistrationConstraint> registration_constraints_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_BACKEND_CONSTRAINT_CONSTRAINT_COLLECTION_H_
