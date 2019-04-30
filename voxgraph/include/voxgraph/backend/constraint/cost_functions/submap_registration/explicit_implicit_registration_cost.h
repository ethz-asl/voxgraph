//
// Created by victor on 24.01.19.
//

#ifndef VOXGRAPH_BACKEND_CONSTRAINT_COST_FUNCTIONS_SUBMAP_REGISTRATION_EXPLICIT_IMPLICIT_REGISTRATION_COST_H_
#define VOXGRAPH_BACKEND_CONSTRAINT_COST_FUNCTIONS_SUBMAP_REGISTRATION_EXPLICIT_IMPLICIT_REGISTRATION_COST_H_

#include <cblox/core/common.h>
#include <cblox/core/submap_collection.h>
#include <cblox/core/tsdf_esdf_submap.h>
#include <cblox/core/tsdf_submap.h>
#include <ceres/ceres.h>
#include <voxblox/interpolator/interpolator.h>
#include "voxgraph/backend/constraint/cost_functions/submap_registration/registration_cost.h"
#include "voxgraph/frontend/submap_collection/voxgraph_submap.h"

namespace voxgraph {
class ExplicitImplicitRegistrationCost : public RegistrationCost {
 public:
  ExplicitImplicitRegistrationCost(
      VoxgraphSubmap::ConstPtr reference_submap_ptr,
      VoxgraphSubmap::ConstPtr reading_submap_ptr, const Config &config);

  bool Evaluate(double const *const *parameters, double *residuals,
                double **jacobians) const override;

 private:
  // Reference to the submap's isosurface vertex vector
  const voxblox::AlignedVector<IsosurfaceVertex> &isosurface_vertices_;
  unsigned int num_isosurface_vertices_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_BACKEND_CONSTRAINT_COST_FUNCTIONS_SUBMAP_REGISTRATION_EXPLICIT_IMPLICIT_REGISTRATION_COST_H_
