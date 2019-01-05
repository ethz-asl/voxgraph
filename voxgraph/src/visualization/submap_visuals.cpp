//
// Created by victor on 19.12.18.
//

#include "voxgraph/visualization/submap_visuals.h"
#include <cblox/mesh/tsdf_submap_mesher.h>
#include <voxblox_ros/mesh_vis.h>
#include <voxblox_ros/ptcloud_vis.h>
#include <string>

namespace voxgraph {
SubmapVisuals::SubmapVisuals(const cblox::TsdfMap::Config &tsdf_map_config)
    : tsdf_submap_mesher_(tsdf_map_config, mesh_config_) {}

void SubmapVisuals::publishMesh(
    const cblox::TsdfSubmapCollection::Ptr &submap_collection_ptr,
    const cblox::SubmapID &submap_id, const cblox::Color &submap_color,
    const std::string &submap_frame, const ros::Publisher &publisher) const {
  // Get a pointer to the submap
  cblox::TsdfSubmap::ConstPtr submap_ptr =
      submap_collection_ptr->getTsdfSubmapConstPtrById(submap_id);
  CHECK_NOTNULL(submap_ptr);

  // Generate the mesh
  cblox::MeshLayer::Ptr mesh_layer_ptr(
      new cblox::MeshLayer(submap_collection_ptr->block_size()));
  voxblox::MeshIntegrator<voxblox::TsdfVoxel> reference_mesh_integrator(
      mesh_config_, submap_ptr->getTsdfMap().getTsdfLayer(),
      mesh_layer_ptr.get());
  reference_mesh_integrator.generateMesh(false, false);
  tsdf_submap_mesher_.colorMeshLayer(submap_color, mesh_layer_ptr.get());

  // Publish mesh
  visualization_msgs::Marker marker;
  voxblox::fillMarkerWithMesh(mesh_layer_ptr, voxblox::ColorMode::kColor,
                              &marker);
  marker.header.frame_id = submap_frame;
  // Update the marker's transform each time its TF frame is updated:
  marker.frame_locked = true;
  publisher.publish(marker);
}

// TODO(victorr): Implement TSDF visualization

}  // namespace voxgraph
