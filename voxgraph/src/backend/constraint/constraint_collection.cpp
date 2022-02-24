#include "voxgraph/backend/constraint/constraint_collection.h"

namespace voxgraph {
void ConstraintCollection::addConstraintsToProblem(
    const NodeCollection& node_collection, ceres::Problem* problem_ptr,
    bool ignore_if_endpoints_constant) {
  CHECK_NOTNULL(problem_ptr);

  // Add all constraints of the appropriate types to the problem
  for (Constraint& absolute_pose_constraint : absolute_pose_constraints_) {
    absolute_pose_constraint.addToProblem(node_collection, problem_ptr,
                                          ignore_if_endpoints_constant);
  }
  for (Constraint& relative_pose_constraint : relative_pose_constraints_) {
    relative_pose_constraint.addToProblem(node_collection, problem_ptr,
                                          ignore_if_endpoints_constant);
  }
  for (Constraint& registration_constraint : registration_constraints_) {
    registration_constraint.addToProblem(node_collection, problem_ptr,
                                         ignore_if_endpoints_constant);
  }
  for (Constraint& planes_constraint : planes_constraints_) {
    planes_constraint.addToProblem(node_collection, problem_ptr,
                                   ignore_if_endpoints_constant);
  }
}
}  // namespace voxgraph
