#ifndef VOXGRAPH_BACKEND_CONSTRAINT_REGISTRATION_CONSTRAINT_H_
#define VOXGRAPH_BACKEND_CONSTRAINT_REGISTRATION_CONSTRAINT_H_

#include <memory>
#include <utility>

#include "voxgraph/backend/constraint/constraint.h"
#include "voxgraph/backend/constraint/cost_functions/registration_cost_function.h"
#include "voxgraph/frontend/submap_collection/voxgraph_submap.h"

namespace voxgraph {
class RegistrationConstraint : public Constraint {
 public:
  typedef std::shared_ptr<RegistrationConstraint> Ptr;
  struct Config : Constraint::Config {
    cblox::SubmapID first_submap_id;
    cblox::SubmapID second_submap_id;
    VoxgraphSubmap::ConstPtr first_submap_ptr;
    VoxgraphSubmap::ConstPtr second_submap_ptr;
    RegistrationCostFunction::Config registration;
  };

  explicit RegistrationConstraint(ConstraintId constraint_id,
                                  const Config& config);

  void addToProblem(const NodeCollection& node_collection,
                    ceres::Problem* problem,
                    bool ignore_if_endpoints_constant) final;

  const Config& getConfig() const { return config_; }

 private:
  const Config config_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_BACKEND_CONSTRAINT_REGISTRATION_CONSTRAINT_H_
