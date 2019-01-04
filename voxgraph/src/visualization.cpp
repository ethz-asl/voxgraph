//
// Created by victor on 19.12.18.
//

#include "voxgraph/visualization.h"
#include <cblox/mesh/tsdf_submap_mesher.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <voxblox_ros/mesh_vis.h>
#include <voxblox_ros/ptcloud_vis.h>
#include <string>

namespace voxgraph {
  Visualization::Visualization(const cblox::TsdfMap::Config& tsdf_map_config) :
    tsdf_submap_mesher_(tsdf_map_config, mesh_config_) {}

  void Visualization::publishMesh(
      cblox::TsdfSubmapCollection::Ptr &submap_collection_ptr,
      const cblox::SubmapID &submap_id, const cblox::Color &submap_color,
      const std::string &submap_frame, const ros::Publisher &publisher) const {
    // Get a pointer to the submap
    cblox::TsdfSubmap::ConstPtr submap_ptr;
    CHECK(submap_collection_ptr->getTsdfSubmapConstPtrById(submap_id,
                                                           submap_ptr));

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

  void Visualization::publishTransform(const voxblox::Transformation &transform,
                                       const std::string &base_frame,
                                       const std::string &target_frame,
                                       bool tf_is_static) const {
    static tf2_ros::TransformBroadcaster transform_broadcaster;
    static tf2_ros::StaticTransformBroadcaster static_transform_broadcaster;

    geometry_msgs::TransformStamped tf_stamped;
    tf_stamped.header.stamp = ros::Time::now();
    tf_stamped.header.frame_id = base_frame;
    tf_stamped.child_frame_id = target_frame;
    tf_stamped.transform.translation.x = transform.getPosition().x();
    tf_stamped.transform.translation.y = transform.getPosition().y();
    tf_stamped.transform.translation.z = transform.getPosition().z();
    tf_stamped.transform.rotation.x = transform.getRotation().x();
    tf_stamped.transform.rotation.y = transform.getRotation().y();
    tf_stamped.transform.rotation.z = transform.getRotation().z();
    tf_stamped.transform.rotation.w = transform.getRotation().w();

    if (tf_is_static) {
      static_transform_broadcaster.sendTransform(tf_stamped);
    } else {
      transform_broadcaster.sendTransform(tf_stamped);
    }
  }

  // TODO(victorr): Implement TSDF visualization
  // TODO(victorr): Move gradient visualization here

  }  // namespace voxgraph
