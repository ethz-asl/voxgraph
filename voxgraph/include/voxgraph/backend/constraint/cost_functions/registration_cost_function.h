#ifndef VOXGRAPH_BACKEND_CONSTRAINT_COST_FUNCTIONS_REGISTRATION_COST_FUNCTION_H_
#define VOXGRAPH_BACKEND_CONSTRAINT_COST_FUNCTIONS_REGISTRATION_COST_FUNCTION_H_

#include <ceres/ceres.h>
#include <ros/ros.h>

#include "voxgraph/frontend/submap_collection/voxgraph_submap.h"
#include "voxgraph/tools/visualization/cost_function_visuals.h"

namespace voxgraph {
class RegistrationCostFunction : public ceres::CostFunction {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum class JacobianEvaluationMethod { kAnalytic = 0, kNumeric };

  struct Config {
    // What type of reference submap points are used
    // to register it to the reading submap
    VoxgraphSubmap::RegistrationPointType registration_point_type =
        VoxgraphSubmap::RegistrationPointType::kIsosurfacePoints;

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

  RegistrationCostFunction(VoxgraphSubmap::ConstPtr reference_submap_ptr,
                           VoxgraphSubmap::ConstPtr reading_submap_ptr,
                           const Config& config);

  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const override;

 protected:
  Config config_;

  // Pointers and const refs to the reading submap that will be aligned
  VoxgraphSubmap::ConstPtr reading_submap_ptr_;
  const voxblox::Layer<voxblox::TsdfVoxel>& reading_tsdf_layer_;
  const voxblox::Layer<voxblox::EsdfVoxel>& reading_esdf_layer_;

  // Reference to registration point sampler
  // NOTE: These points are voxels or isosurface vertices
  //       of the reference submap
  const WeightedSampler<RegistrationPoint>& registration_points_;

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

#endif  // VOXGRAPH_BACKEND_CONSTRAINT_COST_FUNCTIONS_REGISTRATION_COST_FUNCTION_H_
