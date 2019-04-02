//
// Created by victor on 24.01.19.
//

#include "voxgraph/pose_graph/constraint/cost_functions/submap_registration/registration_cost_function_xyz_yaw.h"
#include <minkindr_conversions/kindr_tf.h>
#include <voxblox/interpolator/interpolator.h>
#include <utility>
#include "voxgraph/mapper/submap_collection/voxgraph_submap.h"
#include "voxgraph/tools/tf_helper.h"

namespace voxgraph {
RegistrationCostFunctionXYZYaw::RegistrationCostFunctionXYZYaw(
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

  // Set number of parameters: namely 2 poses, each having 4 params (X,Y,Z,Yaw)
  mutable_parameter_block_sizes()->clear();
  mutable_parameter_block_sizes()->push_back(4);
  mutable_parameter_block_sizes()->push_back(4);

  // Set number of residuals (one per reference submap voxel)
  set_num_residuals(num_relevant_reference_voxels_);
}

bool RegistrationCostFunctionXYZYaw::Evaluate(double const *const *parameters,
                                              double *residuals,
                                              double **jacobians) const {
  unsigned int residual_idx = 0;
  double summed_reference_weight = 0;

  // Get the number of parameters (used when addressing the jacobians array)
  CHECK_EQ(parameter_block_sizes()[0], parameter_block_sizes()[1]);
  int num_params = parameter_block_sizes()[0];

  // Get the reference submap pose from the optimization variables
  voxblox::Transformation::Vector6 T_vec_reference;
  T_vec_reference[0] = parameters[0][0];  // x coordinate in world frame
  T_vec_reference[1] = parameters[0][1];  // y coordinate in world frame
  T_vec_reference[2] = parameters[0][2];  // z coordinate in world frame
  T_vec_reference[3] = 0;
  T_vec_reference[4] = 0;
  T_vec_reference[5] = parameters[0][3];  // yaw angle in world frame
  const voxblox::Transformation T_world__reference =
      voxblox::Transformation::exp(T_vec_reference);

  // Get the reading submap pose from the optimization variables
  voxblox::Transformation::Vector6 T_vec_reading;
  T_vec_reading[0] = parameters[1][0];  // x coordinate in world frame
  T_vec_reading[1] = parameters[1][1];  // y coordinate in world frame
  T_vec_reading[2] = parameters[1][2];  // z coordinate in world frame
  T_vec_reading[3] = 0;
  T_vec_reading[4] = 0;
  T_vec_reading[5] = parameters[1][3];  // yaw angle in world frame
  const voxblox::Transformation T_world__reading =
      voxblox::Transformation::exp(T_vec_reading);

  // Calculate the trigonometric values used when calculating the Jacobians
  float cos_e = std::cos(T_vec_reading[5]);  // cos(yaw_reading)
  float sin_e = std::sin(T_vec_reading[5]);  // sin(yaw_reading)
  // cos(yaw_reading - yaw_reference):
  float cos_emo = std::cos(T_vec_reading[5] - T_vec_reference[5]);
  // sin(yaw_reading - yaw_reference):
  float sin_emo = std::sin(T_vec_reading[5] - T_vec_reference[5]);
  float xe = T_vec_reading[0];    // x_reading
  float ye = T_vec_reading[1];    // y_reading
  float xo = T_vec_reference[0];  // x_reference
  float yo = T_vec_reference[1];  // y_reference

  // Publish the TF corresponding to the current optimized submap pose
  TfHelper::publishTransform(T_world__reading, "world", "optimized_submap",
                             true);

  // Set the relative transform from the reading submap to the reference submap
  const voxblox::Transformation T_reading__reference =
      T_world__reading.inverse() * T_world__reference;

  // Iterate over all reference submap blocks that contain relevant voxels
  for (const auto &kv : reference_block_voxel_list_) {
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
        Eigen::Matrix<float, 1, 4> pResidual_pParamRef, pResidual_pParamRead;
        if (interp_possible) {
          // Calculate q_vector derivatives
          double inv = reference_tsdf_layer_.voxel_size_inv();
          // TODO(victorr): Inv should come from reading_tsdf_layer_,
          //                but when duplicating submaps in cblox
          //                voxel_size_inv doesn't get initialized.
          //                Revert this once duplication works well.

          // Build the Jacobian of the interpolation function
          // over its input vector, evaluated at reading_coordinate
          // Get deltas as used in paper http://spie.org/samples/PM159.pdf
          double Dx = q_vector[1];  // Delta x = (x - x0) / (x1 - x0)
          double Dy = q_vector[2];  // Delta y = (y - y0) / (y1 - y0)
          double Dz = q_vector[3];  // Delta z = (z - z0) / (z1 - z0)

          // Build the Jacobian of the Q vector from the paper
          // clang-format off
          Eigen::Matrix<float, 8, 3> pQ_pr;
          pQ_pr << 0,             0,             0,
                   inv,           0,             0,
                   0,             inv,           0,
                   0,             0,             inv,
                   inv * Dy,      inv * Dx,      0,
                   0,             inv * Dz,      inv * Dy,
                   inv * Dz,      0,             inv * Dx,
                   inv * Dy * Dz, inv * Dx * Dz, inv * Dx * Dy;
          // Calculate the Jacobian of the interpolation function
          Eigen::Matrix<float, 1, 3> pInterp_pr =
              distances * interp_table_.transpose() * pQ_pr;

          // Coordinates of the current point in the reference submap frame
          float xi = reference_coordinate.x();
          float yi = reference_coordinate.y();

          // Jacobian of the transformation T_read_reference * current_point
          // over the reference pose parameters. Note that it is already
          // multiplied with the current point (to avoid getting a 3D tensor)
          Eigen::Matrix<float, 3, 4> pTrr_pParamRef_times_r_ref;
          pTrr_pParamRef_times_r_ref
              << cos_e, sin_e, 0, xi * sin_emo - yi * cos_emo,
                -sin_e, cos_e, 0, xi * cos_emo + yi * sin_emo,
                 0,     0,     1, 0;

          // Jacobian of the transformation T_read_reference * current_point
          // over the reading pose parameters. Note that it is already
          // multiplied with the current point (to avoid getting a 3D tensor)
          Eigen::Matrix<float, 3, 4> pTrr_pParamRead_times_r_ref;
          pTrr_pParamRead_times_r_ref
              << -cos_e, -sin_e, 0, -xi*sin_emo + yi*cos_emo + (xe-xo)*sin_e - (ye-yo)*cos_e,  // NOLINT
                  sin_e, -cos_e, 0, -xi*cos_emo - yi*sin_emo + (xe-xo)*cos_e + (ye-yo)*sin_e,  // NOLINT
                  0,      0,    -1,  0;
          // clang-format on
          // TODO(victorr): Consider using the similarity in
          //                pTrr_pParamRef and pTrr_pParamRead
          //                to reduce redundant computations

          // Compute the Jacobian of the residual over the reference pose params
          pResidual_pParamRef =
              -reference_weight * pInterp_pr * pTrr_pParamRef_times_r_ref;

          // Compute the Jacobian of the residual over the reading pose params
          pResidual_pParamRead =
              -reference_weight * pInterp_pr * pTrr_pParamRead_times_r_ref;
        } else {
          pResidual_pParamRef.setZero();
          pResidual_pParamRead.setZero();
        }
        // Add Jacobian to visualization
        if (options_.visualize_gradients) {
          // Transform the current point and Jacobian into the world frame
          const voxblox::Point world_t_world__point =
              T_world__reading * reading_coordinate;
          voxblox::Point world_jacobian = pResidual_pParamRead.head<3>();
          cost_function_visuals_.addJacobian(world_t_world__point,
                                             world_jacobian);
        }
        // Store the Jacobians for Ceres
        if (jacobians[0] != nullptr) {
          // Jacobians w.r.t. the reference submap pose
          jacobians[0][residual_idx * num_params + 0] = pResidual_pParamRef[0];
          jacobians[0][residual_idx * num_params + 1] = pResidual_pParamRef[1];
          jacobians[0][residual_idx * num_params + 2] = pResidual_pParamRef[2];
          jacobians[0][residual_idx * num_params + 3] = pResidual_pParamRef[3];
        }
        if (jacobians[1] != nullptr) {
          // Jacobians w.r.t. the reading submap pose
          jacobians[1][residual_idx * num_params + 0] = pResidual_pParamRead[0];
          jacobians[1][residual_idx * num_params + 1] = pResidual_pParamRead[1];
          jacobians[1][residual_idx * num_params + 2] = pResidual_pParamRead[2];
          jacobians[1][residual_idx * num_params + 3] = pResidual_pParamRead[3];
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
        jacobians[0][i * num_params + 3] *= factor;
      }
      if (jacobians[1] != nullptr) {
        jacobians[1][i * num_params + 0] *= factor;
        jacobians[1][i * num_params + 1] *= factor;
        jacobians[1][i * num_params + 2] *= factor;
        jacobians[1][i * num_params + 3] *= factor;
      }
    }
  }

  // Scale and publish the visuals, then reset them for the next iteration
  cost_function_visuals_.scaleAndPublish(factor);
  cost_function_visuals_.reset();

  return true;
}
}  // namespace voxgraph
