#include "voxgraph/backend/constraint/constraint.h"

namespace voxgraph {
Constraint::Constraint(Constraint::ConstraintId constraint_id,
                       const Constraint::Config& config)
    : constraint_id_(constraint_id) {
  if (!config.allow_semi_definite_information_matrix) {
    // Compute the square root of the information matrix
    // using Eigen's Cholesky LL^T decomposition
    Eigen::LLT<Eigen::MatrixXd> information_llt(config.information_matrix);
    CHECK(information_llt.info() != Eigen::NumericalIssue)
        << "The square root of the information matrix could not be computed, "
        << "make sure it is symmetric and positive definite: "
        << config.information_matrix;
    sqrt_information_matrix_ = information_llt.matrixL();
  } else {
    // Compute the robust square root of the information matrix
    // using Eigen's LDL^T Cholesky decomposition
    Eigen::LDLT<Eigen::MatrixXd> information_ldlt(config.information_matrix);
    CHECK(information_ldlt.info() != Eigen::NumericalIssue)
        << "The square root of the information matrix could not be computed,"
        << "despite using the robust LDL^T Cholesky decomposition, check: "
        << config.information_matrix;
    CHECK(information_ldlt.isPositive())
        << "The information matrix must be positive semi-definite:"
        << config.information_matrix;

    sqrt_information_matrix_ =
        information_ldlt.transpositionsP().transpose() *
        information_ldlt.matrixL().toDenseMatrix() *
        information_ldlt.vectorD().cwiseSqrt().asDiagonal() *
        information_ldlt.transpositionsP();
    // NOTE: The first permutation term (transpositionsP.transpose) could
    //       be left out, since it cancels once the residual is squared.
    //       However, we leave it in such that the sqrt_information_matrix_
    //       looks familiar to users debugging intermediate steps.
  }
}
}  // namespace voxgraph
