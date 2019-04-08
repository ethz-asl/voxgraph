//
// Created by victor on 08.04.19.
//

#ifndef VOXGRAPH_FRONTEND_MEASUREMENT_PROCESSORS_POINTCLOUD_PROCESSOR_H_
#define VOXGRAPH_FRONTEND_MEASUREMENT_PROCESSORS_POINTCLOUD_PROCESSOR_H_

#include <sensor_msgs/PointCloud2.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <memory>
#include "voxgraph/common.h"
#include "voxgraph/frontend/submap_collection/voxgraph_submap_collection.h"

namespace voxgraph {
class PointcloudProcessor {
 public:
  explicit PointcloudProcessor(
      cblox::SubmapCollection<VoxgraphSubmap>::Ptr submap_collection_ptr,
      bool verbose = false);

  void setTsdfIntegratorConfigFromRosParam(const ros::NodeHandle &node_handle);

  void integratePointcloud(
      const sensor_msgs::PointCloud2::ConstPtr &pointcloud_msg,
      const voxblox::Transformation &T_world_sensor);

 private:
  bool verbose_;

  cblox::SubmapCollection<VoxgraphSubmap>::Ptr submap_collection_ptr_;

  // Tools to integrate the pointclouds into submaps
  voxblox::TsdfIntegratorBase::Config tsdf_integrator_config_;
  std::unique_ptr<voxblox::FastTsdfIntegrator> tsdf_integrator_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_FRONTEND_MEASUREMENT_PROCESSORS_POINTCLOUD_PROCESSOR_H_
