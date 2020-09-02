#include "voxgraph/tools/evaluation/map_evaluation.h"

#include <memory>
#include <string>

#include <ceres/ceres.h>
#include <ros/ros.h>
#include <voxblox/io/layer_io.h>

#include "voxgraph/backend/constraint/cost_functions/registration_cost_function.h"

namespace voxgraph {
MapEvaluation::MapEvaluation(const ros::NodeHandle& node_handle,
                             const std::string& ground_truth_tsdf_file_path)
    : node_handle_(node_handle) {
  // Load the ground truth TSDF layer
  // NOTE: The initial layer will be overwritten but the ptr cannot be null
  auto ground_truth_tsdf_layer_ptr = std::make_shared<TsdfLayer>(1, 1);
  voxblox::io::LoadLayer<TsdfVoxel>(ground_truth_tsdf_file_path,
                                    &ground_truth_tsdf_layer_ptr);
  ground_truth_map_ptr_ = std::make_shared<VoxgraphSubmap>(
      Transformation(), 0, *ground_truth_tsdf_layer_ptr);

  // Advertise the visualization topics
  ground_truth_layer_pub_ =
      node_handle_.advertise<pcl::PointCloud<pcl::PointXYZI>>(
          "tsdf_ground_truth_ptcloud", 1, true);
  ground_truth_mesh_pub_ = node_handle_.advertise<visualization_msgs::Marker>(
      "tsdf_ground_truth_mesh", 1, true);
  rmse_error_pub_ = node_handle_.advertise<pcl::PointCloud<pcl::PointXYZI>>(
      "tsdf_rmse_error", 1, true);
  rmse_error_slice_pub_ =
      node_handle_.advertise<pcl::PointCloud<pcl::PointXYZI>>(
          "tsdf_rmse_error_slice", 1, true);

  // Publish the ground truth TSDF pointcloud
  pcl::PointCloud<pcl::PointXYZI> ground_truth_layer_ptcloud_msg;
  ground_truth_layer_ptcloud_msg.header.frame_id = "odom";
  voxblox::createSurfaceDistancePointcloudFromTsdfLayer(
      *ground_truth_tsdf_layer_ptr, 0.6, &ground_truth_layer_ptcloud_msg);
  ground_truth_layer_pub_.publish(ground_truth_layer_ptcloud_msg);

  // Generate the ground truth TSDF mesh
  auto ground_truth_mesh_layer_ptr = std::make_shared<cblox::MeshLayer>(
      ground_truth_tsdf_layer_ptr->block_size());
  voxblox::MeshIntegrator<voxblox::TsdfVoxel> mesh_integrator(
      voxblox::MeshIntegratorConfig(), *ground_truth_tsdf_layer_ptr,
      ground_truth_mesh_layer_ptr.get());
  mesh_integrator.generateMesh(false, false);

  // Publish the ground truth TSDF mesh
  visualization_msgs::Marker marker;
  voxblox::fillMarkerWithMesh(ground_truth_mesh_layer_ptr,
                              voxblox::ColorMode::kNormals, &marker);
  marker.header.frame_id = "odom";
  ground_truth_mesh_pub_.publish(marker);
}

MapEvaluation::EvaluationDetails MapEvaluation::evaluate(
    const VoxgraphSubmapCollection& submap_collection) {
  // Get the voxel size and number of voxels per side
  voxblox::FloatingPoint voxel_size =
      ground_truth_map_ptr_->getTsdfMap().voxel_size();
  size_t voxels_per_side =
      ground_truth_map_ptr_->getTsdfMap().getTsdfLayer().voxels_per_side();
  CHECK_EQ(submap_collection.getConfig().tsdf_voxel_size, voxel_size)
      << "Submap collection and ground truth must have equal voxel size.";

  // Load the submap collection's projected TSDF map into a VoxgraphSubmap
  // NOTE: This is done in order to use our SubmapRegistrationHelper (see below)
  Transformation T_identity;
  TsdfLayer projected_tsdf_layer(
      submap_collection.getProjectedMap()->getTsdfLayer());
  VoxgraphSubmap::Ptr projected_map_ptr =
      std::make_shared<VoxgraphSubmap>(T_identity, 1, projected_tsdf_layer);

  // Align the ground truth map to the submap collection's projected map
  // NOTE: Moving the submap collection would be more intuitive, but that would
  //       require updating all visuals published during the mapping stage
  projected_map_ptr->finishSubmap();
  ground_truth_map_ptr_->finishSubmap();
  alignSubmapAtoSubmapB(ground_truth_map_ptr_, projected_map_ptr);
  // Apply the transform (interpolate TSDF and ESDF layers into odom frame)
  Transformation T_projected_map__ground_truth =
      ground_truth_map_ptr_->getPose();
  ground_truth_map_ptr_->transformSubmap(T_projected_map__ground_truth);

  // Compute RMSE of the projected w.r.t. the ground truth map
  EsdfLayer error_layer(voxel_size, voxels_per_side);
  voxblox::utils::VoxelEvaluationDetails evaluation_details;
  voxblox::utils::evaluateLayersRmse(
      ground_truth_map_ptr_->getEsdfMap().getEsdfLayer(),
      projected_map_ptr->getEsdfMap().getEsdfLayer(),
      VoxelEvaluationMode::kIgnoreErrorBehindTestSurface, &evaluation_details,
      &error_layer);
  std::cout << evaluation_details.toString() << std::endl;

  // Publish the RMSE error layer for visualization in Rviz
  PointcloudMsg rmse_error_msg;
  PointcloudMsg rmse_error_slice_msg;

  rmse_error_msg.header.frame_id = "odom";
  rmse_error_slice_msg.header.frame_id = "odom";

  voxblox::createDistancePointcloudFromEsdfLayer(error_layer, &rmse_error_msg);
  voxblox::createDistancePointcloudFromEsdfLayerSlice(
      error_layer, 2, 3 * voxel_size, &rmse_error_slice_msg);

  rmse_error_pub_.publish(rmse_error_msg);
  rmse_error_slice_pub_.publish(rmse_error_slice_msg);

  return EvaluationDetails{evaluation_details,
                           T_projected_map__ground_truth.inverse()};
}

void MapEvaluation::alignSubmapAtoSubmapB(
    voxgraph::VoxgraphSubmap::Ptr submap_A,
    voxgraph::VoxgraphSubmap::ConstPtr submap_B) {
  CHECK_NOTNULL(submap_A);
  CHECK_NOTNULL(submap_B);

  // Create and configure Ceres the problem
  ceres::Problem problem;
  ceres::LossFunction* loss_function = nullptr;  // No robust loss function
  ceres::Solver::Summary summary;
  ceres::Solver::Options ceres_options;
  ceres_options.max_num_iterations = 200;
  ceres_options.parameter_tolerance = 1e-12;
  RegistrationCostFunction::Config cost_config;
  cost_config.use_esdf_distance = true;
  cost_config.sampling_ratio = -1;
  cost_config.registration_point_type =
      VoxgraphSubmap::RegistrationPointType::kVoxels;

  // Add the parameter blocks to the optimization
  double layer_B_pose[4] = {0, 0, 0, 0};
  problem.AddParameterBlock(layer_B_pose, 4);
  problem.SetParameterBlockConstant(layer_B_pose);
  double layer_A_pose[4] = {0, 0, 0, 0};
  problem.AddParameterBlock(layer_A_pose, 4);

  // Create and add the submap alignment cost function to the problem
  ceres::CostFunction* cost_function =
      new RegistrationCostFunction(submap_B, submap_A, cost_config);
  problem.AddResidualBlock(cost_function, loss_function, layer_B_pose,
                           layer_A_pose);

  // Run the solver
  ceres::Solve(ceres_options, &problem, &summary);
  std::cout << summary.FullReport() << std::endl;
  std::cout << "Aligned pose: " << layer_A_pose[0] << ", " << layer_A_pose[1]
            << ", " << layer_A_pose[2] << ", " << layer_A_pose[3] << std::endl;

  // Update the pose of submap A
  // clang-format off
  Transformation::Vector6 T_before_after_vec;
  T_before_after_vec << layer_A_pose[0], layer_A_pose[1], layer_A_pose[2],
                        0, 0, layer_A_pose[3];
  submap_A->setPose(Transformation::exp(T_before_after_vec));
  // clang-format on
}
}  // namespace voxgraph
