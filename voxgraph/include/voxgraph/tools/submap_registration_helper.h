#ifndef VOXGRAPH_TOOLS_SUBMAP_REGISTRATION_HELPER_H_
#define VOXGRAPH_TOOLS_SUBMAP_REGISTRATION_HELPER_H_

#include <cblox/core/common.h>
#include <cblox/core/submap_collection.h>
#include <cblox/core/tsdf_esdf_submap.h>
#include <cblox/core/tsdf_submap.h>
#include <ceres/ceres.h>

#include "voxgraph/backend/constraint/cost_functions/registration_cost_function.h"
#include "voxgraph/frontend/submap_collection/voxgraph_submap.h"

namespace voxgraph {
class SubmapRegistrationHelper {
 public:
  struct Options {
    ceres::Solver::Options solver;
    RegistrationCostFunction::Config registration;
  };

  SubmapRegistrationHelper(
      cblox::SubmapCollection<VoxgraphSubmap>::ConstPtr submap_collection_ptr,
      const Options& options);

  ~SubmapRegistrationHelper() = default;

  bool testRegistration(const cblox::SubmapID& reference_submap_id,
                        const cblox::SubmapID& reading_submap_id,
                        double* mission_pose_reading,
                        ceres::Solver::Summary* summary);

 private:
  cblox::SubmapCollection<VoxgraphSubmap>::ConstPtr submap_collection_ptr_;
  Options options_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_TOOLS_SUBMAP_REGISTRATION_HELPER_H_
