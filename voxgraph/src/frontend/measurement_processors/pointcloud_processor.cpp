//
// Created by victor on 09.04.19.
//

#include "voxgraph/frontend/measurement_processors/pointcloud_processor.h"
#include <pcl_conversions/pcl_conversions.h>
#include <voxblox_ros/ros_params.h>
#include <utility>

namespace voxgraph {
PointcloudProcessor::PointcloudProcessor(
    cblox::SubmapCollection<VoxgraphSubmap>::Ptr submap_collection_ptr,
    bool verbose)
    : verbose_(verbose),
      submap_collection_ptr_(std::move(submap_collection_ptr)) {}

void PointcloudProcessor::setTsdfIntegratorConfigFromRosParam(
    const ros::NodeHandle &node_handle) {
  tsdf_integrator_config_ =
      voxblox::getTsdfIntegratorConfigFromRosParam(node_handle);
}

void PointcloudProcessor::integratePointcloud(
    const sensor_msgs::PointCloud2::ConstPtr &pointcloud_msg,
    const voxblox::Transformation &T_world_sensor) {
  // Transform the sensor pose into the submap frame
  const Transformation T_world_submap =
      submap_collection_ptr_->getActiveSubMapPose();
  const Transformation T_submap_sensor =
      T_world_submap.inverse() * T_world_sensor;

  // Convert pointcloud_msg into voxblox::Pointcloud
  pcl::PointCloud<pcl::PointXYZI> pointcloud_pcl;
  pcl::fromROSMsg(*pointcloud_msg, pointcloud_pcl);
  voxblox::Pointcloud pointcloud;
  voxblox::Colors colors;
  pointcloud.reserve(pointcloud_pcl.size());
  colors.reserve(pointcloud_pcl.size());

  // Filter out NaNs while compiling the voxblox pointcloud
  for (const auto &point : pointcloud_pcl.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }

    // Store the point's coordinates
    pointcloud.push_back(voxblox::Point(point.x, point.y, point.z));

    // Map the intensity to a color
    // NOTE: The intensity is a float whose maximum value is sensor dependent.
    //       A scaled 'fast sigmoid' function is therefore used to smoothly map
    //       it's [-Inf, Inf] range to the blue channel with range [0, 255].
    //       Note that other mappings might be better suited, especially when
    //       using a sensor whose intensity is guaranteed to be positive.
    //       This is just quick solution.
    double I = point.intensity;
    I = I / (1 + std::abs(I)) * 127.5 + 127.5;
    auto blue = static_cast<uint8_t>(I);
    colors.push_back(voxblox::Color(0, 0, blue, 1));
  }

  // Initialize the TSDF integrator if this has not yet been done
  if (!tsdf_integrator_) {
    tsdf_integrator_.reset(new voxblox::FastTsdfIntegrator(
        tsdf_integrator_config_,
        submap_collection_ptr_->getActiveTsdfMapPtr()->getTsdfLayerPtr()));
    ROS_INFO("Initialized TSDF Integrator");
  }

  // TODO(victorr): Implement Cartographer style simultaneous integration into
  //                multiple submaps for guaranteed overlap

  // Point the integrator to the current submap
  VoxgraphSubmap::Ptr active_submap_ptr =
      submap_collection_ptr_->getActiveSubMapPtr();
  tsdf_integrator_->setLayer(
      active_submap_ptr->getTsdfMapPtr()->getTsdfLayerPtr());

  // Integrate the pointcloud (and report timings if requested)
  ROS_INFO_COND(verbose_, "Integrating a pointcloud with %lu points.",
                pointcloud.size());
  ros::WallTime start = ros::WallTime::now();
  tsdf_integrator_->integratePointCloud(T_submap_sensor, pointcloud, colors);
  ros::WallTime end = ros::WallTime::now();
  ROS_INFO_COND(
      verbose_,
      "Finished integrating in %f seconds, submap %u now has %lu blocks.",
      (end - start).toSec(), submap_collection_ptr_->getActiveSubMapID(),
      submap_collection_ptr_->getActiveTsdfMap()
          .getTsdfLayer()
          .getNumberOfAllocatedBlocks());

  // Add the current pose to the submap's pose history
  active_submap_ptr->addPoseToHistory(pointcloud_msg->header.stamp,
                                      T_submap_sensor);
}
}  // namespace voxgraph
