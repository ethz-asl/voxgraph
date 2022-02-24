#ifndef VOXGRAPH_BACKEND_CONSTRAINT_CONSTRAINT_COLLECTION_H_
#define VOXGRAPH_BACKEND_CONSTRAINT_CONSTRAINT_COLLECTION_H_

#include <list>

#include "voxgraph/backend/constraint/absolute_pose_constraint.h"
#include "voxgraph/backend/constraint/planes_constraint.h"
#include "voxgraph/backend/constraint/registration_constraint.h"
#include "voxgraph/backend/constraint/relative_pose_constraint.h"

namespace voxgraph {
class ConstraintCollection {
 public:
  typedef std::list<AbsolutePoseConstraint> AbsolutePoseConstraintList;
  typedef std::list<RelativePoseConstraint> RelativePoseConstraintList;
  typedef std::list<RegistrationConstraint> RegistrationConstraintList;
  typedef std::list<PlanesConstraint> PlanesConstraintList;
  // Absolute pose constraints
  void addAbsolutePoseConstraint(const AbsolutePoseConstraint::Config& config) {
    absolute_pose_constraints_.emplace_back(newConstraintId(), config);
  }
  const AbsolutePoseConstraintList& getAbsolutePoseConstraints() {
    return absolute_pose_constraints_;
  }
  void resetAbsolutePoseConstraints() { absolute_pose_constraints_.clear(); }

  // Relative pose constraints
  void addRelativePoseConstraint(const RelativePoseConstraint::Config& config) {
    relative_pose_constraints_.emplace_back(newConstraintId(), config);
  }
  const RelativePoseConstraintList& getRelativePoseConstraints() {
    return relative_pose_constraints_;
  }
  void resetRelativePoseConstraints() { relative_pose_constraints_.clear(); }

  // Registration constraints
  void addRegistrationConstraint(const RegistrationConstraint::Config& config) {
    registration_constraints_.emplace_back(newConstraintId(), config);
  }
  const RegistrationConstraintList& getRegistrationConstraints() {
    return registration_constraints_;
  }
  void resetRegistrationConstraints() { registration_constraints_.clear(); }

  // Planes constraints
  void addPlanesConstraint(const PlanesConstraint::Config& config) {
    planes_constraints_.emplace_back(newConstraintId(), config);
  }
  const PlanesConstraintList& getPlanesConstraints() {
    return planes_constraints_;
  }

  void addConstraintsToProblem(const NodeCollection& node_collection,
                               ceres::Problem* problem_ptr,
                               bool ignore_if_endpoints_constant = true);

 private:
  Constraint::ConstraintId constraint_id_counter_ = 0;
  Constraint::ConstraintId newConstraintId() {
    return constraint_id_counter_++;
  }

  AbsolutePoseConstraintList absolute_pose_constraints_;
  RelativePoseConstraintList relative_pose_constraints_;
  RegistrationConstraintList registration_constraints_;
  PlanesConstraintList planes_constraints_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_BACKEND_CONSTRAINT_CONSTRAINT_COLLECTION_H_
