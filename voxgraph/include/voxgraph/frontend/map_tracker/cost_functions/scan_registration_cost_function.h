#ifndef VOXGRAPH_FRONTEND_MAP_TRACKER_COST_FUNCTIONS_SCAN_REGISTRATION_COST_FUNCTION_H_
#define VOXGRAPH_FRONTEND_MAP_TRACKER_COST_FUNCTIONS_SCAN_REGISTRATION_COST_FUNCTION_H_

#include <ceres/ceres.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <utility>
#include "voxgraph/common.h"

namespace voxgraph {
class ScanRegistrationCostFunction {
 public:
  ScanRegistrationCostFunction(sensor_msgs::PointCloud2::Ptr pointcloud_msg_ptr,
                               VoxgraphSubmap::ConstPtr submap_ptr)
      : pointcloud_msg_ptr_(std::move(pointcloud_msg_ptr)),
        submap_ptr_(submap_ptr),
        tsdf_interpolator_(&submap_ptr->getTsdfMap().getTsdfLayer()),
        voxel_size_inv_(
            submap_ptr->getTsdfMap().getTsdfLayer().voxel_size_inv()) {}

  template <typename T>
  bool operator()(const T* const t_S_C_estimate_ptr,
                  const T* const q_S_C_estimate_ptr,
                  T* residuals_ptr) const;

  static ceres::CostFunction* Create(
      sensor_msgs::PointCloud2::Ptr pointcloud_msg_ptr,
      VoxgraphSubmap::ConstPtr submap_ptr) {
    return new ceres::AutoDiffCostFunction<ScanRegistrationCostFunction,
                                           num_residuals_per_pointcloud_, 3, 4>(
        new ScanRegistrationCostFunction(std::move(pointcloud_msg_ptr),
                                         std::move(submap_ptr)));
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  voxblox::Point getScalarPart(
      const Eigen::Matrix<ceres::Jet<double, 7>, 3, 1> &vector) const {
    // Extract the scalar parts of the Jets, stored in the Jet::a struct member
    voxblox::Point scalar_vector(vector.x().a, vector.y().a, vector.z().a);
    return scalar_vector;
  }

  voxblox::Point getScalarPart(
      const Eigen::Matrix<double, 3, 1> &vector) const {
    return vector.cast<float>();
  }

  template <typename T>
  bool getVoxelsAndJetQVector(const Eigen::Matrix<T, 3, 1>& jet_pos,
                              const voxblox::TsdfVoxel** voxels,
                              Eigen::Matrix<T, 1, 8>* jet_q_vector) const;

  sensor_msgs::PointCloud2::Ptr pointcloud_msg_ptr_;
  VoxgraphSubmap::ConstPtr submap_ptr_;

  voxblox::Interpolator<voxblox::TsdfVoxel> tsdf_interpolator_;
  voxblox::FloatingPoint voxel_size_inv_;

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

  // Hardcode the pointcloud size and decimation rate
  // NOTE: These are hardcoded s.t. an efficient fixed size Ceres cost function
  //       can be used (number of residuals must be known at compile time)
  static constexpr size_t num_points_per_pointcloud__ = 16 * 1450;
  static constexpr int use_every_nth_pointcloud_point_ = 6;
  static constexpr size_t num_residuals_per_pointcloud_ =
      num_points_per_pointcloud__ / use_every_nth_pointcloud_point_;
};
}  // namespace voxgraph

#include "voxgraph/frontend/map_tracker/cost_functions/scan_registration_cost_function_inl.h"

#endif  // VOXGRAPH_FRONTEND_MAP_TRACKER_COST_FUNCTIONS_SCAN_REGISTRATION_COST_FUNCTION_H_
