#include "voxgraph/frontend/measurement_processors/pointcloud_integrator.h"

#include <string>

#include <pcl_conversions/pcl_conversions.h>
#include <voxblox/integrator/projective_tsdf_integrator.h>
#include <voxblox_ros/conversions.h>
#include <voxblox_ros/ros_params.h>

namespace voxgraph {
PointcloudIntegrator::PointcloudIntegrator(bool verbose)
    : verbose_(verbose),
      color_map_(new voxblox::GrayscaleColorMap()),
      tsdf_integrator_method_("fast") {
  // Configure the color map
  color_map_->setMaxValue(10000.0);
}

void PointcloudIntegrator::setTsdfIntegratorConfigFromRosParam(
    const ros::NodeHandle& node_handle) {
  node_handle.param("method", tsdf_integrator_method_, tsdf_integrator_method_);
  tsdf_integrator_config_ =
      voxblox::getTsdfIntegratorConfigFromRosParam(node_handle);
}

void PointcloudIntegrator::integratePointcloud(
    const sensor_msgs::PointCloud2::Ptr& pointcloud_msg,
    const voxblox::Transformation& T_submap_sensor,
    VoxgraphSubmap* submap_ptr) {
  CHECK_NOTNULL(submap_ptr);

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
    tsdf_integrator_ = voxblox::TsdfIntegratorFactory::create(
        tsdf_integrator_method_, tsdf_integrator_config_,
        submap_ptr->getTsdfMapPtr()->getTsdfLayerPtr());

    ROS_INFO_STREAM(
        "Initialized TSDF Integrator of type: " << tsdf_integrator_method_);
    ROS_INFO_STREAM(tsdf_integrator_config_.print());
  }

  // TODO(victorr): Implement optional Cartographer style simultaneous
  //                integration into multiple submaps for guaranteed overlap

  // Point the integrator to the current submap
  tsdf_integrator_->setLayer(submap_ptr->getTsdfMapPtr()->getTsdfLayerPtr());

  // Integrate the pointcloud (and report timings if requested)
  ROS_INFO_COND(verbose_, "Integrating a pointcloud with %lu points.",
                pointcloud.size());
  ros::WallTime start = ros::WallTime::now();
  tsdf_integrator_->integratePointCloud(T_submap_sensor, pointcloud, colors,
                                        /* freespace_points */ false);
  ros::WallTime end = ros::WallTime::now();
  ROS_INFO_COND(
      verbose_,
      "Finished integrating in %f seconds, submap %u now has %lu blocks.",
      (end - start).toSec(), submap_ptr->getID(),
      submap_ptr->getTsdfMap().getTsdfLayer().getNumberOfAllocatedBlocks());
}  // namespace voxgraph
}  // namespace voxgraph
