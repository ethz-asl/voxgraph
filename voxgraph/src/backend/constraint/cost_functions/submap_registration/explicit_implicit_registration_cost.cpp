//
// Created by victor on 24.01.19.
//

#include "voxgraph/backend/constraint/cost_functions/submap_registration/explicit_implicit_registration_cost.h"
#include <minkindr_conversions/kindr_tf.h>
#include <voxblox/interpolator/interpolator.h>
#include <utility>
#include "voxgraph/frontend/submap_collection/voxgraph_submap.h"
#include "voxgraph/tools/tf_helper.h"

namespace voxgraph {
ExplicitImplicitRegistrationCost::ExplicitImplicitRegistrationCost(
    VoxgraphSubmap::ConstPtr reference_submap_ptr,
    VoxgraphSubmap::ConstPtr reading_submap_ptr, const Config &config)
    : RegistrationCost(reference_submap_ptr, reading_submap_ptr, config),
      isosurface_vertices_(reference_submap_ptr->getIsosurfaceVertices()),
      num_isosurface_vertices_(
          reference_submap_ptr->getNumIsosurfaceVertices()) {
  // Set number of residuals (one per isosurface vertex)
  set_num_residuals(num_isosurface_vertices_);
}

// TODO(victorr): Gradually move all common code to the base class
bool ExplicitImplicitRegistrationCost::Evaluate(double const *const *parameters,
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
  if (config_.visualize_transforms_) {
    TfHelper::publishTransform(T_world__reading, "world", "optimized_submap",
                               true);
  }

  // Set the relative transform from the reading submap to the reference submap
  const voxblox::Transformation T_reading__reference =
      T_world__reading.inverse() * T_world__reference;

  // Iterate over all isosurface vertices
  for (const auto &reference_isosurface_vertex : isosurface_vertices_) {
    summed_reference_weight += reference_isosurface_vertex.weight;
    voxblox::Point reference_coordinate = reference_isosurface_vertex.position;

    // Get distances and q_vector in reading submap
    const voxblox::Point reading_coordinate =
        T_reading__reference * reference_coordinate;
    bool interp_possible;
    voxblox::InterpVector distances;
    voxblox::InterpVector q_vector;
    if (config_.use_esdf_distance) {
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
          -reading_distance * reference_isosurface_vertex.weight;
    } else {
      residuals[residual_idx] =
          reference_isosurface_vertex.weight * config_.no_correspondence_cost;
    }

    // Add residual to visualization pointcloud
    if (config_.visualize_residuals) {
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
        double inv = reading_tsdf_layer_.voxel_size_inv();

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
        pResidual_pParamRef = -reference_isosurface_vertex.weight * pInterp_pr *
                              pTrr_pParamRef_times_r_ref;

        // Compute the Jacobian of the residual over the reading pose params
        pResidual_pParamRead = -reference_isosurface_vertex.weight *
                               pInterp_pr * pTrr_pParamRead_times_r_ref;
      } else {
        pResidual_pParamRef.setZero();
        pResidual_pParamRead.setZero();
      }
      // Add Jacobian to visualization
      if (config_.visualize_gradients) {
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

  // Scale residuals by sum of reference voxel weights
  if (summed_reference_weight == 0) return false;
  double factor = num_isosurface_vertices_ / summed_reference_weight;
  for (int i = 0; i < num_isosurface_vertices_; i++) {
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
