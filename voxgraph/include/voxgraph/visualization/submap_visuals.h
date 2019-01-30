//
// Created by victor on 19.12.18.
//

#ifndef VOXGRAPH_VISUALIZATION_SUBMAP_VISUALS_H_
#define VOXGRAPH_VISUALIZATION_SUBMAP_VISUALS_H_

#include <cblox/core/submap_collection.h>
#include <cblox/mesh/submap_mesher.h>
#include <minkindr_conversions/kindr_tf.h>
#include <ros/ros.h>
#include <string>
#include "voxgraph/voxgraph_submap.h"

namespace voxgraph {
class SubmapVisuals {
 public:
  explicit SubmapVisuals(const VoxgraphSubmap::Config &submap_config);

  void publishMesh(const voxblox::MeshLayer::Ptr &mesh_layer_ptr,
                   const std::string &submap_frame,
                   const ros::Publisher &publisher) const;

  void publishMesh(
      const cblox::SubmapCollection<VoxgraphSubmap> &submap_collection,
      const cblox::SubmapID &submap_id, const voxblox::Color &submap_color,
      const std::string &submap_frame, const ros::Publisher &publisher) const;

  void publishSeparatedMesh(
      const cblox::SubmapCollection<VoxgraphSubmap> &submap_collection,
      const std::string &world_frame, const ros::Publisher &publisher);

  void publishCombinedMesh(
      const cblox::SubmapCollection<VoxgraphSubmap> &submap_collection,
      const std::string &world_frame, const ros::Publisher &publisher);

  void publishBox(const VoxgraphSubmap::BoxCornerMatrix &box_corner_matrix,
                  const voxblox::Color &box_color, const std::string &frame_id,
                  const std::string &name_space,
                  const ros::Publisher &publisher) const;

 private:
  voxblox::MeshIntegratorConfig mesh_config_;
  cblox::SubmapMesher submap_mesher_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_VISUALIZATION_SUBMAP_VISUALS_H_
