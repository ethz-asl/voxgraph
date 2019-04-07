//
// Created by victor on 16.01.19.
//

#ifndef VOXGRAPH_BACKEND_CONSTRAINT_ODOMETRY_CONSTRAINT_H_
#define VOXGRAPH_BACKEND_CONSTRAINT_ODOMETRY_CONSTRAINT_H_

#include <memory>
#include "voxgraph/backend/constraint/constraint.h"

namespace voxgraph {
class OdometryConstraint : public Constraint {
 public:
  typedef std::shared_ptr<OdometryConstraint> Ptr;
  struct Config : Constraint::Config {
    cblox::SubmapID origin_submap_id;
    cblox::SubmapID destination_submap_id;
    voxblox::Transformation T_origin_destination;
  };

  OdometryConstraint(ConstraintId constraint_id, const Config& config)
      : Constraint(constraint_id, config), config_(config) {}

  void addToProblem(const NodeCollection& node_collection,
                    ceres::Problem* problem) final;

 private:
  Config config_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_BACKEND_CONSTRAINT_ODOMETRY_CONSTRAINT_H_
