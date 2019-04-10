//
// Created by victor on 03.04.19.
//

#ifndef VOXGRAPH_BACKEND_CONSTRAINT_LOOP_CLOSURE_CONSTRAINT_H_
#define VOXGRAPH_BACKEND_CONSTRAINT_LOOP_CLOSURE_CONSTRAINT_H_

#include <memory>
#include "voxgraph/backend/constraint/constraint.h"

namespace voxgraph {
class LoopClosureConstraint : public Constraint {
 public:
  typedef std::shared_ptr<LoopClosureConstraint> Ptr;
  struct Config : Constraint::Config {
    cblox::SubmapID origin_submap_id;
    cblox::SubmapID destination_submap_id;
    voxblox::Transformation T_origin_destination;
  };

  LoopClosureConstraint(ConstraintId constraint_id, const Config& config)
      : Constraint(constraint_id, config), config_(config) {}

  void addToProblem(const NodeCollection& node_collection,
                    ceres::Problem* problem) final;

 private:
  const Config config_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_BACKEND_CONSTRAINT_LOOP_CLOSURE_CONSTRAINT_H_
