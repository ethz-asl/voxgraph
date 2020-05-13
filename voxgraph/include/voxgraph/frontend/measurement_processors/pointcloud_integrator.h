#ifndef VOXGRAPH_FRONTEND_MEASUREMENT_PROCESSORS_POINTCLOUD_INTEGRATOR_H_
#define VOXGRAPH_FRONTEND_MEASUREMENT_PROCESSORS_POINTCLOUD_INTEGRATOR_H_

#include <memory>
#include <string>

#include <sensor_msgs/PointCloud2.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/utils/color_maps.h>

#include "voxgraph/common.h"
#include "voxgraph/frontend/submap_collection/voxgraph_submap_collection.h"

namespace voxgraph {
class PointcloudIntegrator {
 public:
  explicit PointcloudIntegrator(bool verbose = false);

  void setTsdfIntegratorConfigFromRosParam(const ros::NodeHandle& node_handle);

  void integratePointcloud(const sensor_msgs::PointCloud2::Ptr& pointcloud_msg,
                           const voxblox::Transformation& T_submap_sensor,
                           VoxgraphSubmap* submap_ptr);

 private:
  bool verbose_;

  // Tools used when integrating the pointclouds into submaps
  std::shared_ptr<voxblox::ColorMap> color_map_;
  std::string tsdf_integrator_method_;
  voxblox::TsdfIntegratorBase::Config tsdf_integrator_config_;
  voxblox::TsdfIntegratorBase::Ptr tsdf_integrator_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_FRONTEND_MEASUREMENT_PROCESSORS_POINTCLOUD_INTEGRATOR_H_
