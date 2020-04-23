#ifndef VOXGRAPH_TOOLS_EVALUATION_MAP_EVALUATION_H_
#define VOXGRAPH_TOOLS_EVALUATION_MAP_EVALUATION_H_

#include <string>

#include <voxblox/core/layer.h>
#include <voxblox/utils/evaluation_utils.h>
#include <voxblox_ros/ptcloud_vis.h>

#include "voxgraph/frontend/submap_collection/voxgraph_submap_collection.h"
#include "voxgraph/tools/visualization/submap_visuals.h"

namespace voxgraph {
class MapEvaluation {
 public:
  struct EvaluationDetails {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    voxblox::utils::VoxelEvaluationDetails reconstruction;
    voxblox::Transformation T_ground_truth__reading;
  };

  MapEvaluation(const ros::NodeHandle& nh,
                const std::string& ground_truth_tsdf_file_path);

  MapEvaluation::EvaluationDetails evaluate(
      const VoxgraphSubmapCollection& submap_collection);

  // Find and apply the best rigid body alignment of layer A to B
  void alignSubmapAtoSubmapB(VoxgraphSubmap::Ptr submap_A,
                             VoxgraphSubmap::ConstPtr submap_B);

 private:
  using TsdfVoxel = voxblox::TsdfVoxel;
  using TsdfLayer = voxblox::Layer<TsdfVoxel>;
  using EsdfLayer = voxblox::Layer<voxblox::EsdfVoxel>;
  using VoxelEvaluationMode = voxblox::utils::VoxelEvaluationMode;
  using PointcloudMsg = pcl::PointCloud<pcl::PointXYZI>;

  ros::NodeHandle node_handle_;

  // Ground Truth map
  VoxgraphSubmap::Ptr ground_truth_map_ptr_;

  // ROS publishers for visualization purposes
  ros::Publisher ground_truth_layer_pub_;
  ros::Publisher ground_truth_mesh_pub_;
  ros::Publisher rmse_error_pub_;
  ros::Publisher rmse_error_slice_pub_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_TOOLS_EVALUATION_MAP_EVALUATION_H_
