#include "voxgraph/tools/visualization/submap_visuals.h"

#include <memory>
#include <string>
#include <utility>

#include <cblox/mesh/submap_mesher.h>
#include <eigen_conversions/eigen_msg.h>
#include <voxblox/io/mesh_ply.h>
#include <voxblox_msgs/MultiMesh.h>
#include <voxblox_ros/mesh_vis.h>
#include <voxblox_ros/ptcloud_vis.h>

namespace voxgraph {
SubmapVisuals::SubmapVisuals(VoxgraphSubmap::Config submap_config,
                             voxblox::MeshIntegratorConfig mesh_config)
    : mesh_config_(mesh_config), mesh_opacity_(1.0) {
  // Meshing params from ROS params server
  // NOTE(alexmillane): The separated mesher *requires* color, so this is
  //                    hard-coded.
  combined_submap_mesher_.reset(
      new cblox::SubmapMesher(submap_config, mesh_config_));
  mesh_config_.use_color = true;
  separated_submap_mesher_.reset(
      new cblox::SubmapMesher(submap_config, mesh_config_));
}

void SubmapVisuals::publishMesh(const voxblox::MeshLayer::Ptr& mesh_layer_ptr,
                                const std::string& frame_id,
                                const ros::Publisher& publisher,
                                const voxblox::ColorMode& color_mode) const {
  // Create a marker containing the mesh
  voxblox_msgs::Mesh mesh_msg;
  voxblox::generateVoxbloxMeshMsg(mesh_layer_ptr, color_mode, &mesh_msg);
  mesh_msg.header.frame_id = frame_id;
  publisher.publish(mesh_msg);
}

void SubmapVisuals::publishMultiMesh(
    const voxblox::MeshLayer::Ptr& mesh_layer_ptr, const std::string& frame_id,
    const ros::Publisher& publisher, const voxblox::ColorMode& color_mode,
    const SubmapID mesh_id) const {
  // Create a marker containing the mesh
  voxblox_msgs::MultiMesh mesh_msg;
  voxblox::generateVoxbloxMeshMsg(mesh_layer_ptr, color_mode, &mesh_msg.mesh);
  mesh_msg.header.frame_id = frame_id;
  mesh_msg.name_space = "submap_" + std::to_string(mesh_id);
  publisher.publish(mesh_msg);
}

void SubmapVisuals::publishMesh(
    const cblox::SubmapCollection<VoxgraphSubmap>& submap_collection,
    const cblox::SubmapID& submap_id, const voxblox::Color& submap_color,
    const std::string& frame_id, const ros::Publisher& publisher) const {
  // Get a pointer to the submap
  VoxgraphSubmap::ConstPtr submap_ptr =
      submap_collection.getSubmapConstPtr(submap_id);
  CHECK_NOTNULL(submap_ptr);

  // Generate the mesh
  auto mesh_layer_ptr =
      std::make_shared<cblox::MeshLayer>(submap_collection.block_size());

  voxblox::MeshIntegrator<voxblox::TsdfVoxel> reference_mesh_integrator(
      mesh_config_, submap_ptr->getTsdfMap().getTsdfLayer(),
      mesh_layer_ptr.get());
  reference_mesh_integrator.generateMesh(false, false);
  separated_submap_mesher_->colorMeshLayer(submap_color, mesh_layer_ptr.get());

  // Publish mesh
  publishMultiMesh(mesh_layer_ptr, frame_id, publisher, submap_mesh_color_mode_,
                   submap_id);
}

void SubmapVisuals::publishMesh(
    const cblox::SubmapCollection<VoxgraphSubmap>& submap_collection,
    const SubmapID& submap_id, const std::string& frame_id,
    const ros::Publisher& publisher) {
  const voxblox::Color submap_color =
      submap_id_color_map_.colorLookup(submap_id);
  publishMesh(submap_collection, submap_id, submap_color, frame_id, publisher);
}

void SubmapVisuals::publishSeparatedMesh(
    const cblox::SubmapCollection<VoxgraphSubmap>& submap_collection,
    const std::string& odom_frame, const ros::Publisher& publisher) {
  auto mesh_layer_ptr =
      std::make_shared<cblox::MeshLayer>(submap_collection.block_size());
  separated_submap_mesher_->generateSeparatedMesh(submap_collection,
                                                  mesh_layer_ptr.get());
  publishMesh(mesh_layer_ptr, odom_frame, publisher, submap_mesh_color_mode_);
}

void SubmapVisuals::publishCombinedMesh(
    const cblox::SubmapCollection<VoxgraphSubmap>& submap_collection,
    const std::string& odom_frame, const ros::Publisher& publisher) {
  auto mesh_layer_ptr =
      std::make_shared<cblox::MeshLayer>(submap_collection.block_size());
  combined_submap_mesher_->generateCombinedMesh(submap_collection,
                                                mesh_layer_ptr.get());
  publishMesh(mesh_layer_ptr, odom_frame, publisher, combined_mesh_color_mode_);
}

void SubmapVisuals::saveSeparatedMesh(
    const std::string& filepath,
    const cblox::SubmapCollection<VoxgraphSubmap>& submap_collection) {
  auto mesh_layer_ptr =
      std::make_shared<cblox::MeshLayer>(submap_collection.block_size());
  separated_submap_mesher_->generateSeparatedMesh(submap_collection,
                                                  mesh_layer_ptr.get());
  voxblox::outputMeshLayerAsPly(filepath, *mesh_layer_ptr);
}

void SubmapVisuals::saveCombinedMesh(
    const std::string& filepath,
    const cblox::SubmapCollection<VoxgraphSubmap>& submap_collection) {
  auto mesh_layer_ptr =
      std::make_shared<cblox::MeshLayer>(submap_collection.block_size());
  combined_submap_mesher_->generateCombinedMesh(submap_collection,
                                                mesh_layer_ptr.get());
  voxblox::outputMeshLayerAsPly(filepath, *mesh_layer_ptr);
}

void SubmapVisuals::publishBox(const BoxCornerMatrix& box_corner_matrix,
                               const voxblox::Color& box_color,
                               const std::string& frame_id,
                               const std::string& name_space,
                               const ros::Publisher& publisher) const {
  // Create and configure the marker
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time();
  marker.ns = name_space;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;  // Set to unit quaternion
  marker.scale.x = 0.1;
  voxblox::colorVoxbloxToMsg(box_color, &marker.color);
  marker.frame_locked = false;

  // Add the box edges to the marker's
  std::bitset<3> endpoint_a_idx, endpoint_b_idx;
  for (unsigned int i = 0; i < 8; i++) {
    endpoint_a_idx = std::bitset<3>(i);
    for (unsigned int j = i + 1; j < 8; j++) {
      endpoint_b_idx = std::bitset<3>(j);
      // The endpoints describe an edge of the box if the
      // Hamming distance between both endpoint indices is 1
      if ((endpoint_a_idx ^ endpoint_b_idx).count() == 1) {
        // Add the edge's endpoints to the marker's line list
        geometry_msgs::Point point_msg;
        tf::pointEigenToMsg(box_corner_matrix.col(i).cast<double>(), point_msg);
        marker.points.push_back(point_msg);
        tf::pointEigenToMsg(box_corner_matrix.col(j).cast<double>(), point_msg);
        marker.points.push_back(point_msg);
      }
    }
  }

  // Publish the visualization
  publisher.publish(marker);
}

void SubmapVisuals::publishPoseHistory(
    const VoxgraphSubmapCollection& submap_collection,
    const std::string& odom_frame, const ros::Publisher& publisher) const {
  // Create the pose history message
  nav_msgs::Path pose_history_msg;
  pose_history_msg.header.stamp = ros::Time::now();
  pose_history_msg.header.frame_id = odom_frame;
  pose_history_msg.poses = submap_collection.getPoseHistory();

  // Publish the message
  publisher.publish(pose_history_msg);
}

// TODO(victorr): Implement TSDF visualization

}  // namespace voxgraph
