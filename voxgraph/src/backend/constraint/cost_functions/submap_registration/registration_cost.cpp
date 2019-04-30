//
// Created by victor on 30.04.19.
//

#include "voxgraph/backend/constraint/cost_functions/submap_registration/registration_cost.h"

namespace voxgraph {
RegistrationCost::RegistrationCost(
    VoxgraphSubmap::ConstPtr reference_submap_ptr,
    VoxgraphSubmap::ConstPtr reading_submap_ptr,
    const RegistrationCost::Config &config)
    : ref_submap_ptr_(reference_submap_ptr),
      reading_submap_ptr_(reading_submap_ptr),
      reference_tsdf_layer_(reference_submap_ptr->getTsdfMap().getTsdfLayer()),
      reading_tsdf_layer_(reading_submap_ptr->getTsdfMap().getTsdfLayer()),
      reference_esdf_layer_(reference_submap_ptr->getEsdfMap().getEsdfLayer()),
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
}  // namespace voxgraph
