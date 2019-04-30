//
// Created by victor on 28.04.19.
//

#ifndef VOXGRAPH_BACKEND_CONSTRAINT_COST_FUNCTIONS_SUBMAP_REGISTRATION_REGISTRATION_COST_H_
#define VOXGRAPH_BACKEND_CONSTRAINT_COST_FUNCTIONS_SUBMAP_REGISTRATION_REGISTRATION_COST_H_

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
                   const Config &config)
      : ref_submap_ptr_(reference_submap_ptr),
        reading_submap_ptr_(reading_submap_ptr),
        reference_tsdf_layer_(
            reference_submap_ptr->getTsdfMap().getTsdfLayer()),
        reading_tsdf_layer_(reading_submap_ptr->getTsdfMap().getTsdfLayer()),
        reference_esdf_layer_(
            reference_submap_ptr->getEsdfMap().getEsdfLayer()),
        reading_esdf_layer_(reading_submap_ptr->getEsdfMap().getEsdfLayer()),
        tsdf_interpolator_(&reading_submap_ptr->getTsdfMap().getTsdfLayer()),
        esdf_interpolator_(&reading_submap_ptr->getEsdfMap().getEsdfLayer()),
        config_(config) {
    // Ensure that the reference and reading submaps have gravity aligned Z-axes
    voxblox::Transformation::Vector6 T_vec_reference =
        ref_submap_ptr_->getPose().log();
    voxblox::Transformation::Vector6 T_vec_reading =
        reading_submap_ptr->getPose().log();
    CHECK(T_vec_reference[3] < 1e-6 && T_vec_reference[4] < 1e-6)
        << "Submap Z axes should be gravity aligned, yet submap "
        << ref_submap_ptr_->getID() << " had non-zero roll & pitch: ["
        << T_vec_reference[3] << ", " << T_vec_reference[4] << "]";
    CHECK(T_vec_reading[3] < 1e-6 && T_vec_reading[4] < 1e-6)
        << "Submap Z axes should be gravity aligned, yet submap "
        << reading_submap_ptr_->getID() << " had non-zero roll & pitch: ["
        << T_vec_reading[3] << ", " << T_vec_reading[4] << "]";

    // Set number of parameters: namely 2 poses, each having 4 params
    // (X,Y,Z,Yaw)
    mutable_parameter_block_sizes()->clear();
    mutable_parameter_block_sizes()->push_back(4);
    mutable_parameter_block_sizes()->push_back(4);
  }

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
