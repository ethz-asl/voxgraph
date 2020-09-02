#include "voxgraph/tools/data_servers/submap_server.h"

#include <string>

#include <cblox_msgs/MapLayer.h>
#include <cblox_msgs/MapPoseUpdates.h>
#include <cblox_ros/submap_conversions.h>
#include <minkindr_conversions/kindr_msg.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <voxblox_ros/conversions.h>
#include <voxgraph_msgs/MapSurface.h>

namespace voxgraph {
SubmapServer::SubmapServer(ros::NodeHandle nh_private)
    : frame_names_(FrameNames::fromRosParams(nh_private)) {
  submap_tsdf_pub_ =
      nh_private.advertise<cblox_msgs::MapLayer>("submap_tsdfs", 3, false);
  submap_tsdf_esdf_pub_ =
      nh_private.advertise<cblox_msgs::MapLayer>("submap_esdfs", 3, false);
  submap_surface_pointcloud_pub_ =
      nh_private.advertise<voxgraph_msgs::MapSurface>(
          "submap_surface_pointclouds", 3, false);
  submap_poses_pub_ = nh_private.advertise<cblox_msgs::MapPoseUpdates>(
      "submap_poses", 3, false);
}

void SubmapServer::publishActiveSubmap(
    const VoxgraphSubmapCollection::Ptr& submap_collection_ptr,
    const ros::Time& current_timestamp) {
  // Publish the current (actively mapping) submap
  if (!submap_collection_ptr->empty()) {
    // get active submap
    const VoxgraphSubmap& submap = submap_collection_ptr->getActiveSubmap();
    publishSubmap(submap, current_timestamp);
  }
}

void SubmapServer::publishSubmap(const VoxgraphSubmap& submap,
                                 const ros::Time& timestamp) {
  // Only publish if there are subscribers
  if (submap_tsdf_pub_.getNumSubscribers() > 0) {
    publishSubmapTsdf(submap, frame_names_.output_odom_frame, timestamp,
                      submap_tsdf_pub_);
  }
  if (submap_tsdf_esdf_pub_.getNumSubscribers() > 0) {
    publishSubmapTsdfAndEsdf(submap, frame_names_.output_odom_frame, timestamp,
                             submap_tsdf_esdf_pub_);
  }
  if (submap_surface_pointcloud_pub_.getNumSubscribers() > 0) {
    publishSubmapSurfacePointcloud(submap, frame_names_.output_odom_frame,
                                   timestamp, submap_surface_pointcloud_pub_);
  }
}

void SubmapServer::publishSubmapTsdf(const VoxgraphSubmap& submap,
                                     const ros::Time& timestamp) {
  // Only publish if there are subscribers
  if (submap_tsdf_pub_.getNumSubscribers() > 0) {
    publishSubmapTsdf(submap, frame_names_.output_odom_frame, timestamp,
                      submap_tsdf_pub_);
  }
}

void SubmapServer::publishSubmapTsdfAndEsdf(const VoxgraphSubmap& submap,
                                            const ros::Time& timestamp) {
  // Only publish if there are subscribers
  if (submap_tsdf_esdf_pub_.getNumSubscribers() > 0) {
    publishSubmapTsdfAndEsdf(submap, frame_names_.output_odom_frame, timestamp,
                             submap_tsdf_esdf_pub_);
  }
}

void SubmapServer::publishSubmapSurfacePointcloud(const VoxgraphSubmap& submap,
                                                  const ros::Time& timestamp) {
  // Only publish if there are subscribers
  if (submap_surface_pointcloud_pub_.getNumSubscribers() > 0) {
    publishSubmapSurfacePointcloud(submap, frame_names_.output_odom_frame,
                                   timestamp, submap_surface_pointcloud_pub_);
  }
}

void SubmapServer::publishSubmapTsdf(
    const VoxgraphSubmap& submap, const std::string& frame_id,
    const ros::Time& timestamp, const ros::Publisher& submap_tsdf_publisher) {
  // Create and fill the message
  cblox_msgs::MapLayer submap_tsdf_msg;
  cblox::serializeSubmapToMsg<cblox::TsdfSubmap>(submap, &submap_tsdf_msg);
  submap_tsdf_msg.map_header.pose_estimate.frame_id = frame_id;

  // Publish
  submap_tsdf_publisher.publish(submap_tsdf_msg);
}

void SubmapServer::publishSubmapTsdfAndEsdf(
    const VoxgraphSubmap& submap, const std::string& frame_id,
    const ros::Time& timestamp,
    const ros::Publisher& submap_tsdf_esdf_publisher) {
  // Create and fill the message
  cblox_msgs::MapLayer submap_tsdf_esdf_msg;
  cblox::serializeSubmapToMsg<cblox::TsdfEsdfSubmap>(submap,
                                                     &submap_tsdf_esdf_msg);
  submap_tsdf_esdf_msg.map_header.pose_estimate.frame_id = frame_id;

  // Publish
  submap_tsdf_esdf_publisher.publish(submap_tsdf_esdf_msg);
}

void SubmapServer::publishSubmapSurfacePointcloud(
    const VoxgraphSubmap& submap, const std::string& frame_id,
    const ros::Time& timestamp,
    const ros::Publisher& submap_surface_pointcloud_publisher) {
  // Create the message and set its headers
  voxgraph_msgs::MapSurface submap_surface_pointcloud_msg;
  submap_surface_pointcloud_msg.header =
      cblox::generateHeaderMsg<VoxgraphSubmap>(submap, timestamp);
  submap_surface_pointcloud_msg.map_header =
      cblox::generateSubmapHeaderMsg<VoxgraphSubmap>(submap);
  submap_surface_pointcloud_msg.map_header.pose_estimate.frame_id = frame_id;

  // Get the isosurfaces vertices
  const WeightedSampler<RegistrationPoint>& isosurface_points =
      submap.getRegistrationPoints(
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
  transformKindrToEigen(submap.getPoseHistory().begin()->second.inverse(),
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
  submap_surface_pointcloud_msg.pointcloud.header.stamp = submap.getStartTime();
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
    publishSubmapPoses(submap_collection_ptr, "odom", timestamp,
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
    const VoxgraphSubmap& submap = submap_collection_ptr->getSubmap(submap_id);
    cblox_msgs::MapHeader map_header_msg =
        cblox::generateSubmapHeaderMsg<VoxgraphSubmap>(submap);
    map_header_msg.pose_estimate.frame_id = frame_id;
    pose_msg.map_headers.emplace_back(map_header_msg);
  }

  // Publish message
  submap_poses_publisher.publish(pose_msg);
}

}  // namespace voxgraph
