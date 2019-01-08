//
// Created by victor on 15.12.18.
//

#include "voxgraph/submap_registration/registration_cost_function.h"
#include <minkindr_conversions/kindr_tf.h>
#include <voxblox/interpolator/interpolator.h>
#include <utility>
#include "voxgraph/visualization/tf_helper.h"
#include "voxgraph/voxgraph_submap.h"

namespace voxgraph {
RegistrationCostFunction::RegistrationCostFunction(
    VoxgraphSubmap::ConstPtr reference_submap_ptr,
    VoxgraphSubmap::ConstPtr reading_submap_ptr,
    SubmapRegisterer::Options::CostFunction options)
    : ref_submap_ptr_(reference_submap_ptr),
      reading_submap_ptr_(reading_submap_ptr),
      reference_tsdf_layer_(reference_submap_ptr->getTsdfMap().getTsdfLayer()),
      reading_tsdf_layer_(reading_submap_ptr->getTsdfMap().getTsdfLayer()),
      reference_esdf_layer_(reference_submap_ptr->getEsdfMap().getEsdfLayer()),
      reading_esdf_layer_(reading_submap_ptr->getEsdfMap().getEsdfLayer()),
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

  // Set number of parameters
  mutable_parameter_block_sizes()->clear();
  mutable_parameter_block_sizes()->push_back(3);
  // 3 params (translation around x, y and z)

  // Set number of residuals (one per reference submap voxel)
  set_num_residuals(num_relevant_reference_voxels_);
}

bool RegistrationCostFunction::Evaluate(double const *const *parameters,
                                        double *residuals,
                                        double **jacobians) const {
  unsigned int residual_idx = 0;
  double summed_reference_weight = 0;
  // Set relative transformation
  double const *ref_t_ref_reading = parameters[0];
  int num_params = parameter_block_sizes()[0];
  voxblox::Transformation T_reference__reading =
      ref_submap_ptr_->getPose().inverse() * reading_submap_ptr_->getPose();
  voxblox::Transformation::Vector6 T_vec = T_reference__reading.log();
  T_vec[0] = ref_t_ref_reading[0];
  T_vec[1] = ref_t_ref_reading[1];
  T_vec[2] = ref_t_ref_reading[2];
  T_reference__reading = voxblox::Transformation::exp(T_vec);

  // Publish the TF corresponding to the current optimized submap pose
  voxblox::Transformation T_world__reading =
      ref_submap_ptr_->getPose() * T_reference__reading;
  TfHelper::publishTransform(T_world__reading, "world", "optimized_submap",
                             true);

  // Iterate over all reference submap blocks that contain relevant voxels
  for (const std::pair<voxblox::BlockIndex, voxblox::VoxelIndexList> &kv :
       reference_block_voxel_list_) {
    const voxblox::Block<voxblox::TsdfVoxel> &reference_block =
        reference_tsdf_layer_.getBlockByIndex(kv.first);

    // Iterate over all relevant voxels in block
    for (const voxblox::VoxelIndex &voxel_index : kv.second) {
      CHECK(reference_block.isValidVoxelIndex(voxel_index));
      // Find distance, weight and current coordinates in reference submap
      const voxblox::TsdfVoxel &reference_voxel =
          reference_block.getVoxelByVoxelIndex(voxel_index);
      const double reference_distance = reference_voxel.distance;
      const double reference_weight = reference_voxel.weight;
      summed_reference_weight += reference_weight;
      voxblox::Point reference_coordinate =
          reference_block.computeCoordinatesFromVoxelIndex(voxel_index);

      // Get distances and q_vector in reading submap
      const voxblox::Point reading_pos =
          T_reference__reading.inverse() * reference_coordinate;
      bool interp_possible;
      voxblox::InterpVector distances;
      voxblox::InterpVector q_vector;
      if (options_.use_esdf_distance) {
        const voxblox::EsdfVoxel *neighboring_voxels[8];
        voxblox::Interpolator<voxblox::EsdfVoxel> interpolator(
            &reading_esdf_layer_);
        interp_possible = interpolator.getVoxelsAndQVector(
            reading_pos, neighboring_voxels, &q_vector);
        if (interp_possible) {
          for (int i = 0; i < distances.size(); ++i) {
            distances[i] = static_cast<voxblox::FloatingPoint>(
                neighboring_voxels[i]->distance);
          }
        }
      } else {
        const voxblox::TsdfVoxel *neighboring_voxels[8];
        voxblox::Interpolator<voxblox::TsdfVoxel> interpolator(
            &reading_tsdf_layer_);
        interp_possible = interpolator.getVoxelsAndQVector(
            reading_pos, neighboring_voxels, &q_vector);
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
        voxblox::Point world_t_world__point = T_world__reading * reading_pos;
        cost_function_visuals_.addResidual(world_t_world__point,
                                           residuals[residual_idx]);
      }

      // Calculate Jacobians if requested
      voxblox::Point dResidual;
      if (jacobians != nullptr && jacobians[0] != nullptr) {
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
          // then transform it into the reference submap frame
          dResidual.x() = reference_weight * q_vector_dx *
                          (interp_table_ * distances.transpose());
          dResidual.y() = reference_weight * q_vector_dy *
                          (interp_table_ * distances.transpose());
          dResidual.z() = reference_weight * q_vector_dz *
                          (interp_table_ * distances.transpose());
          dResidual = T_reference__reading * dResidual;
        } else {
          dResidual = {0, 0, 0};
        }
        // Add Jacobian to visualization
        if (options_.visualize_gradients) {
          // Transform the current point and Jacobian into the world frame
          voxblox::Point world_t_world__point = T_world__reading * reading_pos;
          voxblox::Point world_jacobian =
              ref_submap_ptr_->getPose() * dResidual;
          cost_function_visuals_.addJacobian(world_t_world__point,
                                             world_jacobian);
        }
        // Store the Jacobians for Ceres
        jacobians[0][residual_idx * num_params + 0] = dResidual.x();
        jacobians[0][residual_idx * num_params + 1] = dResidual.y();
        jacobians[0][residual_idx * num_params + 2] = dResidual.z();
      }
      residual_idx++;
    }
  }

  // Scale residuals by sum of reference voxel weights
  if (summed_reference_weight == 0) return false;
  double factor = (num_relevant_reference_voxels_ / summed_reference_weight);
  for (int i = 0; i < num_relevant_reference_voxels_; i++) {
    residuals[i] *= factor;
    if (jacobians != nullptr && jacobians[0] != nullptr) {
      jacobians[0][i * num_params + 0] *= factor;
      jacobians[0][i * num_params + 1] *= factor;
      jacobians[0][i * num_params + 2] *= factor;
    }
  }

  // Scale and publish the visuals, then reset it for the next iteration
  cost_function_visuals_.scaleAndPublish(factor);
  cost_function_visuals_.reset();

  return true;
}
}  // namespace voxgraph
