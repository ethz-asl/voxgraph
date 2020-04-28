#include "voxgraph/tools/data_servers/submap_server.h"

#include <minkindr_conversions/kindr_msg.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <voxblox_ros/conversions.h>
#include <cblox_msgs/MapPoseUpdates.h>
#include <cblox_msgs/MapLayer.h>
#include <voxgraph_msgs/MapSurface.h>
#include <voxgraph/tools/data_servers/submap_conversions.h>

namespace voxgraph {
SubmapServer::SubmapServer(ros::NodeHandle nh_private) {
  submap_tsdf_pub_ =
      nh_private.advertise<cblox_msgs::MapLayer>("submap_tsdfs", 3, false);
  submap_esdf_pub_ =
      nh_private.advertise<cblox_msgs::MapLayer>("submap_esdfs", 3, false);
  submap_surface_pointcloud_pub_ =
      nh_private.advertise<voxgraph_msgs::MapSurface>(
          "submap_surface_pointclouds", 3, false);
  submap_poses_pub_ = nh_private.advertise<cblox_msgs::MapPoseUpdates>(
          "submap_poses", 3, false);
}

cblox_msgs::MapLayer SubmapServer::serializeActiveSubmap(
    const VoxgraphSubmapCollection::Ptr& submap_collection_ptr,
    const ros::Time& current_timestamp) {
  // Publish the current (actively mapping) submap
  cblox_msgs::MapLayer submap_tsdf_msg;
  if (!submap_collection_ptr->empty()) {
    // get active submap
    const voxgraph::VoxgraphSubmap &submap =
        submap_collection_ptr->getActiveSubmap();

    // Create the message and set its headers
    submap_tsdf_msg.header = generateHeaderMsg(submap, current_timestamp);
    submap_tsdf_msg.map_header = generateSubmapHeaderMsg(submap);

    // Set the message's TSDF
    voxblox::serializeLayerAsMsg<voxblox::TsdfVoxel>(
        submap.getTsdfMap().getTsdfLayer(), false, &submap_tsdf_msg.tsdf_layer);
    submap_tsdf_msg.tsdf_layer.action =
        static_cast<uint8_t>(voxblox::MapDerializationAction::kReset);
  }
  return submap_tsdf_msg;
}

void SubmapServer::publishActiveSubmap(
    const VoxgraphSubmapCollection::Ptr& submap_collection_ptr,
    const ros::Time& current_timestamp) {
  // Publish the current (actively mapping) submap
  if (!submap_collection_ptr->empty()) {
    // get active submap
    const voxgraph::VoxgraphSubmap::Ptr& submap_ptr =
        submap_collection_ptr->getActiveSubmapPtr();
    publishSubmap(submap_ptr, current_timestamp);
  }
}

void SubmapServer::publishSubmap(const VoxgraphSubmap::Ptr& submap_ptr,
                                 const ros::Time& timestamp) {
  // Only publish if there are subscribers
  if (submap_tsdf_pub_.getNumSubscribers() > 0) {
    publishSubmapTsdf(submap_ptr, timestamp, submap_tsdf_pub_);
  }
  if (submap_esdf_pub_.getNumSubscribers() > 0) {
    publishSubmapTsdfAndEsdf(submap_ptr, timestamp, submap_esdf_pub_);
  }
  if (submap_surface_pointcloud_pub_.getNumSubscribers() > 0) {
    publishSubmapSurfacePointcloud(submap_ptr, timestamp,
                                   submap_surface_pointcloud_pub_);
  }
}

void SubmapServer::publishSubmapTsdf(const voxgraph::VoxgraphSubmap::Ptr& submap_ptr,
                                     const ros::Time& timestamp) {
  // Only publish if there are subscribers
  if (submap_tsdf_pub_.getNumSubscribers() > 0) {
    publishSubmapTsdf(submap_ptr, timestamp, submap_tsdf_pub_);
  }
}

void SubmapServer::publishSubmapTsdfAndEsdf(const voxgraph::VoxgraphSubmap::Ptr& submap_ptr,
                                            const ros::Time& timestamp) {
  // Only publish if there are subscribers
  if (submap_esdf_pub_.getNumSubscribers() > 0) {
    publishSubmapTsdfAndEsdf(submap_ptr, timestamp, submap_esdf_pub_);
  }
}

void SubmapServer::publishSubmapSurfacePointcloud(
    const voxgraph::VoxgraphSubmap::Ptr& submap_ptr, const ros::Time& timestamp) {
  // Only publish if there are subscribers
  if (submap_surface_pointcloud_pub_.getNumSubscribers() > 0) {
    publishSubmapSurfacePointcloud(submap_ptr, timestamp,
                                   submap_surface_pointcloud_pub_);
  }
}

void SubmapServer::publishSubmapTsdf(
    const VoxgraphSubmap::Ptr& submap_ptr, const ros::Time& timestamp,
    const ros::Publisher& submap_tsdf_publisher) {
  // Create and fill the message
  cblox_msgs::MapLayer submap_tsdf_msg;
  cblox::serializeSubmapToMsg<cblox::TsdfSubmap>(submap_ptr, &submap_tsdf_msg);
  submap_tsdf_msg.map_header =
      cblox::generateSubmapHeaderMsg<VoxgraphSubmap>(submap_ptr);

  // Publish
  submap_tsdf_publisher.publish(submap_tsdf_msg);
}

void SubmapServer::publishSubmapTsdfAndEsdf(
    const VoxgraphSubmap::Ptr& submap_ptr, const ros::Time& timestamp,
    const ros::Publisher& submap_esdf_publisher) {
  // Create and fill the message
  cblox_msgs::MapLayer submap_esdf_msg;
  cblox::serializeSubmapToMsg<cblox::TsdfEsdfSubmap>(submap_ptr, &submap_esdf_msg);
  submap_esdf_msg.map_header =
      cblox::generateSubmapHeaderMsg<VoxgraphSubmap>(submap_ptr);

  // Publish
  submap_esdf_publisher.publish(submap_esdf_msg);
}

void SubmapServer::publishSubmapSurfacePointcloud(
    const VoxgraphSubmap::Ptr& submap_ptr, const ros::Time& timestamp,
    const ros::Publisher& submap_surface_pointcloud_publisher) {
  // Create the message and set its headers
  voxgraph_msgs::MapSurface submap_surface_pointcloud_msg;
  submap_surface_pointcloud_msg.header =
      cblox::generateHeaderMsg<VoxgraphSubmap>(submap_ptr, timestamp);
  submap_surface_pointcloud_msg.map_header =
      cblox::generateSubmapHeaderMsg<VoxgraphSubmap>(submap_ptr);

  // Get the isosurfaces vertices
  const WeightedSampler<RegistrationPoint>& isosurface_points =
      submap_ptr->getRegistrationPoints(
          VoxgraphSubmap::RegistrationPointType::kIsosurfacePoints);

  // Allocate a PCL pointcloud used as an intermediate step
  // when converting to ROS' PointCloud2 type
  pcl::PointCloud<pcl::PointXYZI> pcl_surface_pointcloud;
  pcl_surface_pointcloud.reserve(isosurface_points.size());

  // Get the pose of the robot at the time that the submap was created,
  // in case we want to transform the pointcloud from the 4DoF submap
  // origin frame to the 6DoF robot base_link frame
  // NOTE: The PointCloud2.header.stamp is set to the submap creation timestamp
  Eigen::Affine3f T_B_S;
  transformKindrToEigen(submap_ptr->getPoseHistory().begin()->second.inverse(),
                        &T_B_S);

  // Add the isosurface vertices to the PCL pointclouds
  // TODO(victorr): Implement weighted subsampling for low bandwidth scenarios
  for (size_t point_i = 0; point_i < isosurface_points.size(); point_i++) {
    pcl::PointXYZI pcl_isosurface_point;
    pcl_isosurface_point.x = isosurface_points[point_i].position.x();
    pcl_isosurface_point.y = isosurface_points[point_i].position.y();
    pcl_isosurface_point.z = isosurface_points[point_i].position.z();
    pcl_isosurface_point.intensity = isosurface_points[point_i].weight;
    if (fake_6dof_transforms_) {
      // Transform the points from the 4DoF submap frame
      // to the 6DoF base_link frame
      pcl_isosurface_point = pcl::transformPoint(pcl_isosurface_point, T_B_S);
    }
    pcl_surface_pointcloud.push_back(pcl_isosurface_point);
  }

  // Store the pointcloud in the msg's PointCloud2 field and set its header
  pcl::toROSMsg(pcl_surface_pointcloud,
                submap_surface_pointcloud_msg.pointcloud);
  submap_surface_pointcloud_msg.pointcloud.header.stamp = submap_ptr->getStartTime();
  submap_surface_pointcloud_msg.pointcloud.header.frame_id = "imu";
  // TODO(victorr): Document which timestamps and frames are used where
  // TODO(victorr): Parameterize the frame names

  // Publish
  submap_surface_pointcloud_publisher.publish(submap_surface_pointcloud_msg);
}

void SubmapServer::publishSubmapPoses(
    const VoxgraphSubmapCollection::Ptr& submap_collection_ptr,
    const ros::Time& timestamp) {
  if (submap_poses_pub_.getNumSubscribers() > 0) {
    publishSubmapPoses(submap_collection_ptr, "mission", timestamp,
                       submap_poses_pub_);
  }
}

void SubmapServer::publishSubmapPoses(
    const VoxgraphSubmapCollection::Ptr& submap_collection_ptr,
    const std::string& frame_id, const ros::Time& timestamp,
    const ros::Publisher& submap_poses_publisher) {
  // prep map header
  cblox_msgs::MapPoseUpdates pose_msg;
  pose_msg.header.frame_id = frame_id;
  pose_msg.header.stamp = ros::Time::now();
  // fill pose array
  for (const SubmapID submap_id : submap_collection_ptr->getIDs()) {
    const VoxgraphSubmap::Ptr& submap_ptr =
        submap_collection_ptr->getSubmapPtr(submap_id);
    pose_msg.map_headers.emplace_back(
        cblox::generateSubmapHeaderMsg<VoxgraphSubmap>(submap_ptr));
  }

  // Publish message
  submap_poses_publisher.publish(pose_msg);
}

}  // namespace voxgraph
