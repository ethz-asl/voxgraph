//
// Created by victor on 15.12.18.
//

#include "voxgraph/backend/constraint/cost_functions/submap_registration/registration_cost_function_xyz.h"
#include <minkindr_conversions/kindr_tf.h>
#include <voxblox/interpolator/interpolator.h>
#include <utility>
#include "voxgraph/frontend/submap_collection/voxgraph_submap.h"
#include "voxgraph/tools/tf_helper.h"

namespace voxgraph {
RegistrationCostFunctionXYZ::RegistrationCostFunctionXYZ(
    VoxgraphSubmap::ConstPtr reference_submap_ptr,
    VoxgraphSubmap::ConstPtr reading_submap_ptr,
    SubmapRegisterer::Options::CostFunction options)
    : ref_submap_ptr_(reference_submap_ptr),
      reading_submap_ptr_(reading_submap_ptr),
      reference_tsdf_layer_(reference_submap_ptr->getTsdfMap().getTsdfLayer()),
      reading_tsdf_layer_(reading_submap_ptr->getTsdfMap().getTsdfLayer()),
      reference_esdf_layer_(reference_submap_ptr->getEsdfMap().getEsdfLayer()),
      reading_esdf_layer_(reading_submap_ptr->getEsdfMap().getEsdfLayer()),
      tsdf_interpolator_(&reading_submap_ptr->getTsdfMap().getTsdfLayer()),
      esdf_interpolator_(&reading_submap_ptr->getEsdfMap().getEsdfLayer()),
      options_(options) {
  // Get list of all allocated voxel blocks in the reference submap
  voxblox::BlockIndexList ref_block_list;
  reference_tsdf_layer_.getAllAllocatedBlocks(&ref_block_list);

  // Calculate the number of voxels per block
  size_t vps = reference_tsdf_layer_.voxels_per_side();
  size_t num_voxels_per_block = vps * vps * vps;

  // Create a list containing only reference voxels that matter
  // i.e. that have been observed and fall within a truncation band
  // Iterate over all allocated blocks in the reference submap
  for (const voxblox::BlockIndex &ref_block_index : ref_block_list) {
    const voxblox::Block<voxblox::TsdfVoxel> &reference_block =
        reference_tsdf_layer_.getBlockByIndex(ref_block_index);
    // Iterate over all voxels in block
    for (size_t linear_index = 0u; linear_index < num_voxels_per_block;
         ++linear_index) {
      const voxblox::TsdfVoxel &ref_voxel =
          reference_block.getVoxelByLinearIndex(linear_index);
      // Select the observed voxels within the truncation band
      if (ref_voxel.weight > options_.min_voxel_weight &&
          std::abs(ref_voxel.distance) < options_.max_voxel_distance) {
        num_relevant_reference_voxels_++;
        voxblox::VoxelIndex voxel_index =
            reference_block.computeVoxelIndexFromLinearIndex(linear_index);
        reference_block_voxel_list_[ref_block_index].push_back(voxel_index);
      }
    }
  }

  // Set number of parameters: namely 2 poses, each having 3 params (X,Y,Z)
  mutable_parameter_block_sizes()->clear();
  mutable_parameter_block_sizes()->push_back(3);
  mutable_parameter_block_sizes()->push_back(3);

  // Set number of residuals (one per reference submap voxel)
  set_num_residuals(num_relevant_reference_voxels_);
}

bool RegistrationCostFunctionXYZ::Evaluate(double const *const *parameters,
                                           double *residuals,
                                           double **jacobians) const {
  unsigned int residual_idx = 0;
  double summed_reference_weight = 0;

  // Get the number of parameters (used when addressing the jacobians array)
  CHECK_EQ(parameter_block_sizes()[0], parameter_block_sizes()[1]);
  int num_params = parameter_block_sizes()[0];

  // Get the reference submap pose from the optimization variables
  double const *world_t_world__reference = parameters[0];
  voxblox::Transformation::Vector6 T_vec_reference =
      ref_submap_ptr_->getPose().log();
  T_vec_reference[0] = world_t_world__reference[0];
  T_vec_reference[1] = world_t_world__reference[1];
  T_vec_reference[2] = world_t_world__reference[2];
  const voxblox::Transformation T_world__reference =
      voxblox::Transformation::exp(T_vec_reference);

  // Get the reading submap pose from the optimization variables
  double const *world_t_world__reading = parameters[1];
  voxblox::Transformation::Vector6 T_vec_reading =
      reading_submap_ptr_->getPose().log();
  T_vec_reading[0] = world_t_world__reading[0];
  T_vec_reading[1] = world_t_world__reading[1];
  T_vec_reading[2] = world_t_world__reading[2];
  const voxblox::Transformation T_world__reading =
      voxblox::Transformation::exp(T_vec_reading);

  // Publish the TF corresponding to the current optimized submap pose
  // TODO(victorr): Parametrize this
  if (false) {
    TfHelper::publishTransform(T_world__reading, "world", "optimized_submap",
                               true);
  }

  // Set the relative transform from the reading submap to the reference submap
  const voxblox::Transformation T_reading__reference =
      T_world__reading.inverse() * T_world__reference;

  // Iterate over all reference submap blocks that contain relevant voxels
  for (const std::pair<voxblox::BlockIndex, voxblox::VoxelIndexList> &kv :
       reference_block_voxel_list_) {
    // Get a const ref to the current reference submap tsdf block
    const voxblox::Block<voxblox::TsdfVoxel> &reference_tsdf_block =
        reference_tsdf_layer_.getBlockByIndex(kv.first);
    // Get a const pointer to the current reference submap esdf block, if needed
    voxblox::Block<voxblox::EsdfVoxel>::ConstPtr reference_esdf_block;
    if (options_.use_esdf_distance) {
      reference_esdf_block = reference_esdf_layer_.getBlockPtrByIndex(kv.first);
    }

    // Iterate over all relevant voxels in block
    for (const voxblox::VoxelIndex &voxel_index : kv.second) {
      CHECK(reference_tsdf_block.isValidVoxelIndex(voxel_index));
      // Find weight and current coordinates in reference submap
      const voxblox::TsdfVoxel &reference_tsdf_voxel =
          reference_tsdf_block.getVoxelByVoxelIndex(voxel_index);
      const double reference_weight = reference_tsdf_voxel.weight;
      summed_reference_weight += reference_weight;
      voxblox::Point reference_coordinate =
          reference_tsdf_block.computeCoordinatesFromVoxelIndex(voxel_index);
      // Find distance in reference submap
      double reference_distance;
      if (options_.use_esdf_distance) {
        CHECK(reference_esdf_block->isValidVoxelIndex(voxel_index));
        const voxblox::EsdfVoxel &reference_esdf_voxel =
            reference_esdf_block->getVoxelByVoxelIndex(voxel_index);
        reference_distance = reference_esdf_voxel.distance;
      } else {
        reference_distance = reference_tsdf_voxel.distance;
      }

      // Get distances and q_vector in reading submap
      const voxblox::Point reading_coordinate =
          T_reading__reference * reference_coordinate;
      bool interp_possible;
      voxblox::InterpVector distances;
      voxblox::InterpVector q_vector;
      if (options_.use_esdf_distance) {
        const voxblox::EsdfVoxel *neighboring_voxels[8];
        interp_possible = esdf_interpolator_.getVoxelsAndQVector(
            reading_coordinate, neighboring_voxels, &q_vector);
        if (interp_possible) {
          for (int i = 0; i < distances.size(); ++i) {
            distances[i] = static_cast<voxblox::FloatingPoint>(
                neighboring_voxels[i]->distance);
          }
        }
      } else {
        const voxblox::TsdfVoxel *neighboring_voxels[8];
        interp_possible = tsdf_interpolator_.getVoxelsAndQVector(
            reading_coordinate, neighboring_voxels, &q_vector);
        if (interp_possible) {
          for (int i = 0; i < distances.size(); ++i) {
            distances[i] = static_cast<voxblox::FloatingPoint>(
                neighboring_voxels[i]->distance);
          }
        }
      }

      // Add residual
      if (interp_possible) {
        // Interpolate distance
        const double reading_distance =
            q_vector * (interp_table_ * distances.transpose());

        residuals[residual_idx] =
            (reference_distance - reading_distance) * reference_weight;
      } else {
        residuals[residual_idx] =
            reference_weight * options_.no_correspondence_cost;
      }

      // Add residual to visualization pointcloud
      if (options_.visualize_residuals) {
        // Transform the current point into the world frame
        voxblox::Point world_t_world__point =
            T_world__reading * reading_coordinate;
        cost_function_visuals_.addResidual(world_t_world__point,
                                           residuals[residual_idx]);
      }

      // Calculate Jacobians if requested
      if (jacobians != nullptr) {
        voxblox::Point dResidual;
        if (interp_possible) {
          // Calculate q_vector derivatives
          double inv = reference_tsdf_layer_.voxel_size_inv();
          // TODO(victorr): Inv should come from reading_tsdf_layer_,
          // but when duplicating submaps in cblox voxel_size_inv
          // doesn't get initialized. Revert this once duplication works well.
          double delta_x = q_vector[1];
          double delta_y = q_vector[2];
          double delta_z = q_vector[3];
          voxblox::InterpVector q_vector_dx;
          voxblox::InterpVector q_vector_dy;
          voxblox::InterpVector q_vector_dz;
          q_vector_dx << 0, inv, 0, 0, inv * delta_y, 0, inv * delta_z,
              inv * delta_y * delta_z;
          q_vector_dy << 0, 0, inv, 0, inv * delta_x, inv * delta_z, 0,
              inv * delta_x * delta_z;
          q_vector_dz << 0, 0, 0, inv, 0, inv * delta_y, inv * delta_x,
              inv * delta_x * delta_y;
          // Compute the Jacobian in the reading submap's frame of reference,
          // then transform it into the world frame
          dResidual.x() = reference_weight * q_vector_dx *
                          (interp_table_ * distances.transpose());
          dResidual.y() = reference_weight * q_vector_dy *
                          (interp_table_ * distances.transpose());
          dResidual.z() = reference_weight * q_vector_dz *
                          (interp_table_ * distances.transpose());
          dResidual = T_world__reading.getRotationMatrix() * dResidual;
        } else {
          dResidual.setZero();
        }
        // Add Jacobian to visualization
        if (options_.visualize_gradients) {
          // Transform the current point and Jacobian into the world frame
          const voxblox::Point world_t_world__point =
              T_world__reading * reading_coordinate;
          const voxblox::Point world_jacobian = dResidual;
          cost_function_visuals_.addJacobian(world_t_world__point,
                                             world_jacobian);
        }
        // Store the Jacobians for Ceres
        if (jacobians[0] != nullptr) {
          // Jacobians w.r.t. the reference submap pose
          jacobians[0][residual_idx * num_params + 0] = -dResidual.x();
          jacobians[0][residual_idx * num_params + 1] = -dResidual.y();
          jacobians[0][residual_idx * num_params + 2] = -dResidual.z();
        }
        if (jacobians[1] != nullptr) {
          // Jacobians w.r.t. the reading submap pose
          jacobians[1][residual_idx * num_params + 0] = dResidual.x();
          jacobians[1][residual_idx * num_params + 1] = dResidual.y();
          jacobians[1][residual_idx * num_params + 2] = dResidual.z();
        }
      }
      residual_idx++;
    }
  }

  // Scale residuals by sum of reference voxel weights
  if (summed_reference_weight == 0) return false;
  double factor = (num_relevant_reference_voxels_ / summed_reference_weight);
  for (int i = 0; i < num_relevant_reference_voxels_; i++) {
    residuals[i] *= factor;
    if (jacobians != nullptr) {
      if (jacobians[0] != nullptr) {
        jacobians[0][i * num_params + 0] *= factor;
        jacobians[0][i * num_params + 1] *= factor;
        jacobians[0][i * num_params + 2] *= factor;
      }
      if (jacobians[1] != nullptr) {
        jacobians[1][i * num_params + 0] *= factor;
        jacobians[1][i * num_params + 1] *= factor;
        jacobians[1][i * num_params + 2] *= factor;
      }
    }
  }

  // Scale and publish the visuals, then reset them for the next iteration
  cost_function_visuals_.scaleAndPublish(factor);
  cost_function_visuals_.reset();

  return true;
}
}  // namespace voxgraph
