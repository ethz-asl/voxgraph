//
// Created by victor on 10.04.19.
//

#include "voxgraph/tools/evaluation/map_evaluation.h"
#include <ceres/ceres.h>
#include <ros/ros.h>
#include <voxblox/integrator/merge_integration.h>
#include <voxblox/io/layer_io.h>
#include <memory>
#include <string>
#include "voxgraph/backend/constraint/cost_functions/submap_registration/registration_cost_function_xyz_yaw.h"

namespace voxgraph {
MapEvaluation::MapEvaluation(const ros::NodeHandle &node_handle,
                             const std::string &ground_truth_tsdf_file_path)
    : node_handle_(node_handle) {
  // Load the ground truth TSDF layer
  // NOTE: The initial layer will be overwritten but the ptr cannot be null
  ground_truth_tsdf_layer_ptr_ = std::make_shared<TsdfLayer>(1, 1);
  voxblox::io::LoadLayer<TsdfVoxel>(ground_truth_tsdf_file_path,
                                    &ground_truth_tsdf_layer_ptr_);

  // Advertise the visualization topics
  ground_truth_layer_pub_ =
      node_handle_.advertise<pcl::PointCloud<pcl::PointXYZI>>(
          "tsdf_ground_truth", 1, true);
  rmse_error_pub_ = node_handle_.advertise<pcl::PointCloud<pcl::PointXYZI>>(
      "tsdf_rmse_error", 1, true);
  rmse_error_slice_pub_ =
      node_handle_.advertise<pcl::PointCloud<pcl::PointXYZI>>(
          "tsdf_rmse_error_slice", 1, true);
  rmse_error_surface_distance_pub_ =
      node_handle_.advertise<pcl::PointCloud<pcl::PointXYZI>>(
          "tsdf_rmse_error_surface_distance", 1, true);

  // Publish the ground truth TSDF layer visualization
  pcl::PointCloud<pcl::PointXYZI> ground_truth_layer_msg;
  ground_truth_layer_msg.header.frame_id = "world";
  voxblox::createSurfaceDistancePointcloudFromTsdfLayer(
      *ground_truth_tsdf_layer_ptr_, 0.6, &ground_truth_layer_msg);
  ground_truth_layer_pub_.publish(ground_truth_layer_msg);
}

void MapEvaluation::alignLayerAtoLayerB(TsdfLayer *layer_A,
                                        const TsdfLayer &layer_B) {
  CHECK_NOTNULL(layer_A);

  // Create and configure Ceres the problem
  ceres::Problem problem;
  ceres::LossFunction *loss_function = nullptr;
  ceres::Solver::Summary summary;
  ceres::Solver::Options ceres_options;
  SubmapRegisterer::Options::CostFunction cost_function_options;
  cost_function_options.use_esdf_distance = false;

  // Add the parameter blocks to the optimization
  double layer_B_pose[4] = {0, 0, 0, 0};
  problem.AddParameterBlock(layer_B_pose, 4);
  problem.SetParameterBlockConstant(layer_B_pose);
  double layer_A_pose[4] = {0, 0, 0, 0};
  problem.AddParameterBlock(layer_A_pose, 4);

  // Create a VoxgraphSubmap for each layer, to interface
  // with voxgraph::RegistrationCostFunctionXYZYaw()
  auto map_A_ptr =
      std::make_shared<VoxgraphSubmap>(Transformation(), 0, *layer_A);
  auto map_B_ptr =
      std::make_shared<VoxgraphSubmap>(Transformation(), 0, layer_B);

  // Create and add the submap alignment cost function to the problem
  ceres::CostFunction *cost_function = new RegistrationCostFunctionXYZYaw(
      map_B_ptr, map_A_ptr, cost_function_options);
  problem.AddResidualBlock(cost_function, loss_function, layer_B_pose,
                           layer_A_pose);

  // Run the solver
  ceres::Solve(ceres_options, &problem, &summary);
  std::cout << summary.FullReport() << std::endl;
  //  std::cout << "Ground truth pose: " << layer_B_pose[0] << ", "
  //            << layer_B_pose[1] << ", " << layer_B_pose[2] << ", "
  //            << layer_B_pose[3] << std::endl;
  std::cout << "Projected map pose: " << layer_A_pose[0] << ", "
            << layer_A_pose[1] << ", " << layer_A_pose[2] << ", "
            << layer_A_pose[3] << std::endl;

  // Align layer A
  // clang-format off
  Transformation::Vector6 T_vec;
  T_vec << layer_A_pose[0], layer_A_pose[1], layer_A_pose[2],
           0, 0, layer_A_pose[3];
  Transformation T_before_after = Transformation::exp(T_vec);
  voxblox::transformLayer(map_A_ptr->getTsdfMap().getTsdfLayer(),
                          T_before_after.inverse(), layer_A);
  // clang-format on
}

void MapEvaluation::evaluate(
    const VoxgraphSubmapCollection &submap_collection) {
  // Load the submap collection's projected TSDF map into a VoxgraphSubmap
  // NOTE: This is done in order to use voxgraph's SubmapRegisterer (see below)
  TsdfLayer projected_tsdf_layer(
      submap_collection.getProjectedMap()->getTsdfLayer());

  // Align the submap collection's projected map to the ground truth
  alignLayerAtoLayerB(&projected_tsdf_layer, *ground_truth_tsdf_layer_ptr_);

  // Compute RMSE of the projected w.r.t. the ground truth map
  voxblox::FloatingPoint voxel_size =
      ground_truth_tsdf_layer_ptr_->voxel_size();
  size_t voxels_per_side = ground_truth_tsdf_layer_ptr_->voxels_per_side();
  TsdfLayer error_layer(voxel_size, voxels_per_side);
  voxblox::utils::VoxelEvaluationDetails evaluation_details;
  voxblox::utils::evaluateLayersRmse(*ground_truth_tsdf_layer_ptr_,
                                     projected_tsdf_layer,
                                     VoxelEvaluationMode::kEvaluateAllVoxels,
                                     &evaluation_details, &error_layer);
  std::cout << evaluation_details.toString() << std::endl;

  // Publish the RMSE error layer for visualization in Rviz
  PointcloudMsg rmse_error_msg;
  PointcloudMsg rmse_error_slice_msg;
  PointcloudMsg rmse_error_surface_distance_msg;

  rmse_error_msg.header.frame_id = "world";
  rmse_error_slice_msg.header.frame_id = "world";
  rmse_error_surface_distance_msg.header.frame_id = "world";

  voxblox::createDistancePointcloudFromTsdfLayer(error_layer, &rmse_error_msg);
  voxblox::createDistancePointcloudFromTsdfLayerSlice(
      error_layer, 2, 3 * voxel_size, &rmse_error_slice_msg);
  voxblox::createSurfaceDistancePointcloudFromTsdfLayer(
      error_layer, 3 * voxel_size, &rmse_error_surface_distance_msg);

  rmse_error_pub_.publish(rmse_error_msg);
  rmse_error_slice_pub_.publish(rmse_error_slice_msg);
  rmse_error_surface_distance_pub_.publish(rmse_error_surface_distance_msg);
}
}  // namespace voxgraph
