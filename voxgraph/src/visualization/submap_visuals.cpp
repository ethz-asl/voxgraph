//
// Created by victor on 19.12.18.
//

#include "voxgraph/visualization/submap_visuals.h"
#include <cblox/mesh/submap_mesher.h>
#include <voxblox_ros/mesh_vis.h>
#include <voxblox_ros/ptcloud_vis.h>
#include <memory>
#include <string>

namespace voxgraph {
SubmapVisuals::SubmapVisuals(const VoxgraphSubmap::Config &submap_config)
    : submap_mesher_(submap_config, mesh_config_) {}

void SubmapVisuals::publishMesh(const voxblox::MeshLayer::Ptr &mesh_layer_ptr,
                                const std::string &submap_frame,
                                const ros::Publisher &publisher) const {
  // Publish a marker representing the mesh
  visualization_msgs::Marker marker;
  voxblox::fillMarkerWithMesh(mesh_layer_ptr, voxblox::ColorMode::kColor,
                              &marker);
  marker.header.frame_id = submap_frame;
  // Update the marker's transform each time its TF frame is updated:
  marker.frame_locked = true;
  publisher.publish(marker);
}

void SubmapVisuals::publishMesh(
    const cblox::SubmapCollection<VoxgraphSubmap> &submap_collection,
    const cblox::SubmapID &submap_id, const cblox::Color &submap_color,
    const std::string &submap_frame, const ros::Publisher &publisher) const {
  // Get a pointer to the submap
  VoxgraphSubmap::ConstPtr submap_ptr =
      submap_collection.getSubMapConstPtrById(submap_id);
  CHECK_NOTNULL(submap_ptr);

  // Generate the mesh
  auto mesh_layer_ptr =
      std::make_shared<cblox::MeshLayer>(submap_collection.block_size());
  voxblox::MeshIntegrator<voxblox::TsdfVoxel> reference_mesh_integrator(
      mesh_config_, submap_ptr->getTsdfMap().getTsdfLayer(),
      mesh_layer_ptr.get());
  reference_mesh_integrator.generateMesh(false, false);
  submap_mesher_.colorMeshLayer(submap_color, mesh_layer_ptr.get());

  // Publish mesh
  publishMesh(mesh_layer_ptr, submap_frame, publisher);
}

void SubmapVisuals::publishSeparatedMesh(
    const cblox::SubmapCollection<VoxgraphSubmap> &submap_collection,
    const std::string &submap_frame, const ros::Publisher &publisher) {
  auto mesh_layer_ptr =
      std::make_shared<cblox::MeshLayer>(submap_collection.block_size());
  submap_mesher_.generateSeparatedMesh(submap_collection, mesh_layer_ptr.get());
  publishMesh(mesh_layer_ptr, submap_frame, publisher);
}

// TODO(victorr): Implement TSDF visualization

}  // namespace voxgraph
