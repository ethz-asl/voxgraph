//
// Created by victor on 09.04.19.
//

#include "voxgraph/backend/constraint/constraint_collection.h"

namespace voxgraph {
void ConstraintCollection::addAllToProblem(
    const NodeCollection &node_collection, ceres::Problem *problem_ptr) {
  CHECK_NOTNULL(problem_ptr);

  // Add all constraints of all types to the problem
  for (Constraint &absolute_pose_constraint : absolute_pose_constraints_) {
    absolute_pose_constraint.addToProblem(node_collection, problem_ptr);
  }
  for (Constraint &relative_pose_constraint : relative_pose_constraints_) {
    relative_pose_constraint.addToProblem(node_collection, problem_ptr);
  }
  for (Constraint &registration_constraint : registration_constraints_) {
    registration_constraint.addToProblem(node_collection, problem_ptr);
  }
}
}  // namespace voxgraph
