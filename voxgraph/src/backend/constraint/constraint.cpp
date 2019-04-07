//
// Created by victor on 03.04.19.
//

#include "voxgraph/backend/constraint/constraint.h"

namespace voxgraph {
Constraint::Constraint(Constraint::ConstraintId constraint_id,
                       const Constraint::Config &config)
    : constraint_id_(constraint_id) {
  // Compute the square root of the information matrix
  // using Eigen's Cholesky LL^T decomposition
  Eigen::LLT<Eigen::MatrixXd> information_llt(config.information_matrix);
  CHECK(information_llt.info() != Eigen::NumericalIssue)
      << "The square root of the information matrix could not be computed, "
      << "make sure it is symmetric and positive definite.";
  sqrt_information_matrix_ = information_llt.matrixL();
}
}  // namespace voxgraph
