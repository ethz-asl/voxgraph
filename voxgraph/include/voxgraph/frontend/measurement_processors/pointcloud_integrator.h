#ifndef VOXGRAPH_FRONTEND_MEASUREMENT_PROCESSORS_POINTCLOUD_INTEGRATOR_H_
#define VOXGRAPH_FRONTEND_MEASUREMENT_PROCESSORS_POINTCLOUD_INTEGRATOR_H_

#include <sensor_msgs/PointCloud2.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/utils/color_maps.h>
#include <memory>
#include "voxgraph/common.h"
#include "voxgraph/frontend/submap_collection/voxgraph_submap_collection.h"

namespace voxgraph {
class PointcloudIntegrator {
 public:
  explicit PointcloudIntegrator(
      cblox::SubmapCollection<VoxgraphSubmap>::Ptr submap_collection_ptr,
      bool verbose = false);

  void setTsdfIntegratorConfigFromRosParam(const ros::NodeHandle &node_handle);

  void integratePointcloud(const sensor_msgs::PointCloud2::Ptr &pointcloud_msg,
                           const voxblox::Transformation &T_mission_sensor);

 private:
  bool verbose_;

  cblox::SubmapCollection<VoxgraphSubmap>::Ptr submap_collection_ptr_;

  // Tools used when integrating the pointclouds into submaps
  std::shared_ptr<voxblox::ColorMap> color_map_;
  voxblox::TsdfIntegratorBase::Config tsdf_integrator_config_;
  std::unique_ptr<voxblox::FastTsdfIntegrator> tsdf_integrator_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_FRONTEND_MEASUREMENT_PROCESSORS_POINTCLOUD_INTEGRATOR_H_
