//
// Created by victor on 10.04.19.
//

#include "voxgraph/tools/evaluation/map_evaluation.h"
#include <ros/ros.h>
#include <voxblox/io/layer_io.h>
#include <string>

namespace voxgraph {
MapEvaluation::MapEvaluation(const ros::NodeHandle &node_handle,
                             const std::string &ground_truth_tsdf_file_path)
    : node_handle_(node_handle) {
  // Load the ground truth TSDF layer
  voxblox::io::LoadLayer<TsdfVoxel>(ground_truth_tsdf_file_path,
                                    &ground_truth_tsdf_layer_ptr_);
  voxel_size_ = ground_truth_tsdf_layer_ptr_->voxel_size();
  voxels_per_side_ = ground_truth_tsdf_layer_ptr_->voxels_per_side();

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

void MapEvaluation::evaluate(
    const VoxgraphSubmapCollection &submap_collection) {
  // Get the projected TSDF map from the submap collection
  voxblox::TsdfMap::Ptr projected_tsdf_map =
      submap_collection.getProjectedMap();

  // Compute RMSE of the projected w.r.t. the ground truth map
  TsdfLayer error_layer(voxel_size_, voxels_per_side_);
  voxblox::utils::VoxelEvaluationDetails evaluation_details;
  voxblox::utils::evaluateLayersRmse(*ground_truth_tsdf_layer_ptr_,
                                     *projected_tsdf_map->getTsdfLayerPtr(),
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
      error_layer, 2, 3 * voxel_size_, &rmse_error_slice_msg);
  voxblox::createSurfaceDistancePointcloudFromTsdfLayer(
      error_layer, 3 * voxel_size_, &rmse_error_surface_distance_msg);

  rmse_error_pub_.publish(rmse_error_msg);
  rmse_error_slice_pub_.publish(rmse_error_slice_msg);
  rmse_error_surface_distance_pub_.publish(rmse_error_surface_distance_msg);
}
}  // namespace voxgraph
