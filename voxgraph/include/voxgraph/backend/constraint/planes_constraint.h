#ifndef VOXGRAPH_BACKEND_CONSTRAINT_PLANES_CONSTRAINT_H_
#define VOXGRAPH_BACKEND_CONSTRAINT_PLANES_CONSTRAINT_H_

#include <memory>

#include "voxgraph/backend/constraint/constraint.h"
#include "voxgraph/frontend/plane_collection/plane_type.h"
#include "voxgraph/backend/constraint/cost_functions/planes_cost_function.h"

namespace voxgraph {
class PlanesConstraint : public Constraint {
 public:
  typedef std::shared_ptr<PlanesConstraint> Ptr;
  struct Config : Constraint::Config {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    cblox::SubmapID origin_submap_id;
    cblox::SubmapID destination_submap_id;
    voxblox::Transformation T_origin_destination;
    voxblox::Transformation T_M_R_origin;
    voxblox::Transformation T_M_R_destination;
    PlaneType::ConstPtr origin_plane;
    PlaneType::ConstPtr destination_plane;
    PlanesCostFunction::Config planes_cost_config;
  };

  PlanesConstraint(ConstraintId constraint_id, const Config& config)
      : Constraint(constraint_id, config), config_(config) {}

  void addToProblem(const NodeCollection& node_collection,
                    ceres::Problem* problem,
                    bool ignore_if_endpoints_constant) final;

  const Config& getConfig() const { return config_; }

 private:
  const Config config_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_BACKEND_CONSTRAINT_PLANES_CONSTRAINT_H_
