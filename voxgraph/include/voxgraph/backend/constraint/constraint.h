#ifndef VOXGRAPH_BACKEND_CONSTRAINT_CONSTRAINT_H_
#define VOXGRAPH_BACKEND_CONSTRAINT_CONSTRAINT_H_

#include <memory>

#include <ceres/ceres.h>

#include "voxgraph/backend/node/node_collection.h"

namespace voxgraph {
class Constraint {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<Constraint> Ptr;
  typedef unsigned int ConstraintId;
  typedef Eigen::Matrix<double, 4, 4> InformationMatrix;

  struct Config {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    InformationMatrix information_matrix;
    bool allow_semi_definite_information_matrix = false;
  };

  explicit Constraint(ConstraintId constraint_id, const Config& config);
  virtual ~Constraint() = default;

  virtual void addToProblem(const NodeCollection& node_collection,
                            ceres::Problem* problem) = 0;

  const ceres::ResidualBlockId getResidualBlockId() {
    return residual_block_id_;
  }

 protected:
  static constexpr ceres::LossFunction* kNoRobustLossFunction = nullptr;

  const ConstraintId constraint_id_;
  ceres::ResidualBlockId residual_block_id_ = nullptr;

  InformationMatrix sqrt_information_matrix_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_BACKEND_CONSTRAINT_CONSTRAINT_H_
