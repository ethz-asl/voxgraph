//
// Created by victor on 10.04.19.
//

#ifndef VOXGRAPH_TOOLS_EVALUATION_MAP_EVALUATION_H_
#define VOXGRAPH_TOOLS_EVALUATION_MAP_EVALUATION_H_

#include <voxblox/core/layer.h>
#include <voxblox/utils/evaluation_utils.h>
#include <voxblox_ros/ptcloud_vis.h>
#include <string>
#include "voxgraph/frontend/submap_collection/voxgraph_submap_collection.h"

namespace voxgraph {
class MapEvaluation {
 public:
  using TsdfVoxel = voxblox::TsdfVoxel;
  using TsdfLayer = voxblox::Layer<TsdfVoxel>;

  MapEvaluation(const ros::NodeHandle &nh,
                const std::string &ground_truth_tsdf_file_path);

  // Find and apply the best rigid body alignment of layer A to B
  void alignLayerAtoLayerB(TsdfLayer *layer_A, const TsdfLayer &layer_B);

  void evaluate(const VoxgraphSubmapCollection &submap_collection);

 private:
  using VoxelEvaluationMode = voxblox::utils::VoxelEvaluationMode;
  using PointcloudMsg = pcl::PointCloud<pcl::PointXYZI>;

  ros::NodeHandle node_handle_;

  // Ground Truth map
  TsdfLayer::Ptr ground_truth_tsdf_layer_ptr_;

  // ROS publishers for visualization purposes
  ros::Publisher ground_truth_layer_pub_;
  ros::Publisher rmse_error_pub_;
  ros::Publisher rmse_error_slice_pub_;
  ros::Publisher rmse_error_surface_distance_pub_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_TOOLS_EVALUATION_MAP_EVALUATION_H_
