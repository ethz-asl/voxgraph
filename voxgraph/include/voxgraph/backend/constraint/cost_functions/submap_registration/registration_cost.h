//
// Created by victor on 28.04.19.
//

#ifndef VOXGRAPH_BACKEND_CONSTRAINT_COST_FUNCTIONS_SUBMAP_REGISTRATION_REGISTRATION_COST_H_
#define VOXGRAPH_BACKEND_CONSTRAINT_COST_FUNCTIONS_SUBMAP_REGISTRATION_REGISTRATION_COST_H_

#include <ceres/ceres.h>
#include "voxgraph/frontend/submap_collection/voxgraph_submap.h"

// For visualization only
#include <ros/ros.h>
#include "voxgraph/tools/visualization/cost_function_visuals.h"

namespace voxgraph {
class RegistrationCost : public ceres::CostFunction {
 public:
  enum class RegistrationMethod {
    kImplicitToImplicit = 0,
    kExplicitToImplicit
  };

  enum class JacobianEvaluationMethod { kAnalytic = 0, kNumeric };

  struct Config {
    // What method to use register the submaps to each other
    RegistrationMethod registration_method =
        RegistrationMethod::kImplicitToImplicit;

    // What method to use to calculate the Jacobians
    JacobianEvaluationMethod jacobian_evaluation_method =
        JacobianEvaluationMethod::kAnalytic;

    // Sampling ratio, set to -1 to disable down sampling
    float sampling_ratio = -1;

    // Cost to assign for voxels that can't be interpolated
    // i.e. that don't exist in the other submap
    double no_correspondence_cost = 0;

    // Whether to use the TSDF or ESDF distance
    bool use_esdf_distance = true;

    // Visuals for debugging purposes
    bool visualize_residuals = false;
    bool visualize_gradients = false;
    bool visualize_transforms_ = false;
  };

  RegistrationCost(VoxgraphSubmap::ConstPtr reference_submap_ptr,
                   VoxgraphSubmap::ConstPtr reading_submap_ptr,
                   const Config &config);

 protected:
  Config config_;

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

#endif  // VOXGRAPH_BACKEND_CONSTRAINT_COST_FUNCTIONS_SUBMAP_REGISTRATION_REGISTRATION_COST_H_
