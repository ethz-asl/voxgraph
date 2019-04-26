//
// Created by victor on 15.12.18.
//

#ifndef VOXGRAPH_BACKEND_CONSTRAINT_COST_FUNCTIONS_SUBMAP_REGISTRATION_CORRELATIVE_COST_FUNCTION_3_DOF_H_
#define VOXGRAPH_BACKEND_CONSTRAINT_COST_FUNCTIONS_SUBMAP_REGISTRATION_CORRELATIVE_COST_FUNCTION_3_DOF_H_

#include <cblox/core/common.h>
#include <cblox/core/submap_collection.h>
#include <cblox/core/tsdf_esdf_submap.h>
#include <cblox/core/tsdf_submap.h>
#include <ceres/ceres.h>
#include <voxblox/interpolator/interpolator.h>
#include "voxgraph/backend/constraint/cost_functions/submap_registration/submap_registerer.h"

// For visualization only
#include <ros/ros.h>
#include "voxgraph/tools/visualization/cost_function_visuals.h"

namespace voxgraph {
// TODO(victorr): Create and inherint from parent class
//                to reduce code duplication w.r.t. xyz_yaw version
class CorrelativeCostFunction3DoF : public ceres::CostFunction {
 public:
  CorrelativeCostFunction3DoF(VoxgraphSubmap::ConstPtr reference_submap_ptr,
                              VoxgraphSubmap::ConstPtr reading_submap_ptr,
                              SubmapRegisterer::Options::CostFunction options);

  bool Evaluate(double const *const *parameters, double *residuals,
                double **jacobians) const override;

  const unsigned int getNumRelevantVoxels() const {
    return num_relevant_reference_voxels_;
  }

 private:
  // Cost function options
  SubmapRegisterer::Options::CostFunction options_;

  // Pointers and const refs to the submaps that will be aligned
  VoxgraphSubmap::ConstPtr ref_submap_ptr_;
  VoxgraphSubmap::ConstPtr reading_submap_ptr_;
  const voxblox::Layer<voxblox::TsdfVoxel> &reference_tsdf_layer_;
  const voxblox::Layer<voxblox::TsdfVoxel> &reading_tsdf_layer_;
  const voxblox::Layer<voxblox::EsdfVoxel> &reference_esdf_layer_;
  const voxblox::Layer<voxblox::EsdfVoxel> &reading_esdf_layer_;

  // Interpolators
  voxblox::Interpolator<voxblox::TsdfVoxel> tsdf_interpolator_;
  voxblox::Interpolator<voxblox::EsdfVoxel> esdf_interpolator_;

  // Block and voxel index hash map storing
  // only the relevant voxels (observed and within truncation distance)
  voxblox::HierarchicalIndexMap reference_block_voxel_list_;
  unsigned int num_relevant_reference_voxels_ = 0;

  // Used for residual and Jacobian visualization
  mutable CostFunctionVisuals cost_function_visuals_;

  // This matrix is used to interpolate voxels
  // It corresponds to matrix B_1 from paper: http://spie.org/samples/PM159.pdf
  // clang-format off
  const voxblox::InterpTable interp_table_ =
      (voxblox::InterpTable() << 1,  0,  0,  0,  0,  0,  0,  0,
                                -1,  0,  0,  0,  1,  0,  0,  0,
                                -1,  0,  1,  0,  0,  0,  0,  0,
                                -1,  1,  0,  0,  0,  0,  0,  0,
                                 1,  0, -1,  0, -1,  0,  1,  0,
                                 1, -1, -1,  1,  0,  0,  0,  0,
                                 1, -1,  0,  0, -1,  1,  0,  0,
                                -1,  1,  1, -1,  1, -1, -1,  1).finished();
  // clang-format on
};
}  // namespace voxgraph

#endif  // VOXGRAPH_BACKEND_CONSTRAINT_COST_FUNCTIONS_SUBMAP_REGISTRATION_CORRELATIVE_COST_FUNCTION_3_DOF_H_
