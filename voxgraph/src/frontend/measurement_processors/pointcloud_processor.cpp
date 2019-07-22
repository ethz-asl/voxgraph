#include "voxgraph/frontend/measurement_processors/pointcloud_processor.h"
#include <pcl_conversions/pcl_conversions.h>
#include <voxblox_ros/conversions.h>
#include <voxblox_ros/ros_params.h>
#include <string>
#include <utility>

namespace voxgraph {
PointcloudProcessor::PointcloudProcessor(
    cblox::SubmapCollection<VoxgraphSubmap>::Ptr submap_collection_ptr,
    bool verbose)
    : verbose_(verbose),
      submap_collection_ptr_(std::move(submap_collection_ptr)),
      color_map_(new voxblox::RainbowColorMap()) {
  // Configure the color map
  color_map_->setMaxValue(100.0);
}

void PointcloudProcessor::setTsdfIntegratorConfigFromRosParam(
    const ros::NodeHandle &node_handle) {
  tsdf_integrator_config_ =
      voxblox::getTsdfIntegratorConfigFromRosParam(node_handle);
}

void PointcloudProcessor::integratePointcloud(
    const sensor_msgs::PointCloud2::Ptr &pointcloud_msg,
    const voxblox::Transformation &T_mission_sensor) {
  // Transform the sensor pose into the submap frame
  const Transformation T_mission_submap =
      submap_collection_ptr_->getActiveSubmapPose();
  const Transformation T_submap_sensor =
      T_mission_submap.inverse() * T_mission_sensor;

  // Convert the pointcloud msg into a voxblox::Pointcloud
  voxblox::Pointcloud pointcloud;
  voxblox::Colors colors;
  // Detect the PCL pointcloud type
  bool color_pointcloud = false;
  bool has_intensity = false;
  for (size_t d = 0; d < pointcloud_msg->fields.size(); ++d) {
    if (pointcloud_msg->fields[d].name == std::string("rgb")) {
      color_pointcloud = true;
      // Quick hack to fix PCL color parsing
      pointcloud_msg->fields[d].datatype = sensor_msgs::PointField::FLOAT32;
    } else if (pointcloud_msg->fields[d].name == std::string("intensity")) {
      has_intensity = true;
    }
  }
  // Convert differently depending on RGB or I type
  if (color_pointcloud) {
    pcl::PointCloud<pcl::PointXYZRGB> pointcloud_pcl;
    // pointcloud_pcl is modified below:
    pcl::fromROSMsg(*pointcloud_msg, pointcloud_pcl);
    voxblox::convertPointcloud(pointcloud_pcl, color_map_, &pointcloud,
                               &colors);
  } else if (has_intensity) {
    pcl::PointCloud<pcl::PointXYZI> pointcloud_pcl;
    // pointcloud_pcl is modified below:
    pcl::fromROSMsg(*pointcloud_msg, pointcloud_pcl);
    voxblox::convertPointcloud(pointcloud_pcl, color_map_, &pointcloud,
                               &colors);
  } else {
    pcl::PointCloud<pcl::PointXYZ> pointcloud_pcl;
    // pointcloud_pcl is modified below:
    pcl::fromROSMsg(*pointcloud_msg, pointcloud_pcl);
    voxblox::convertPointcloud(pointcloud_pcl, color_map_, &pointcloud,
                               &colors);
  }

  // Initialize the TSDF integrator if this has not yet been done
  if (!tsdf_integrator_) {
    tsdf_integrator_.reset(new voxblox::FastTsdfIntegrator(
        tsdf_integrator_config_,
        submap_collection_ptr_->getActiveTsdfMapPtr()->getTsdfLayerPtr()));
    ROS_INFO("Initialized TSDF Integrator");
  }

  // TODO(victorr): Implement optional Cartographer style simultaneous
  //                integration into multiple submaps for guaranteed overlap

  // Point the integrator to the current submap
  VoxgraphSubmap::Ptr active_submap_ptr =
      submap_collection_ptr_->getActiveSubmapPtr();
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
      (end - start).toSec(), submap_collection_ptr_->getActiveSubmapID(),
      submap_collection_ptr_->getActiveTsdfMap()
          .getTsdfLayer()
          .getNumberOfAllocatedBlocks());
}
}  // namespace voxgraph
