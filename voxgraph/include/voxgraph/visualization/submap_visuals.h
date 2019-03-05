//
// Created by victor on 19.12.18.
//

#ifndef VOXGRAPH_VISUALIZATION_SUBMAP_VISUALS_H_
#define VOXGRAPH_VISUALIZATION_SUBMAP_VISUALS_H_

#include <cblox/core/tsdf_submap.h>
#include <cblox/mesh/tsdf_submap_mesher.h>
#include <minkindr_conversions/kindr_tf.h>
#include <ros/ros.h>
#include <string>

namespace voxgraph {
class SubmapVisuals {
 public:
  explicit SubmapVisuals(const cblox::TsdfMap::Config &tsdf_map_config);

  void publishMesh(
      const cblox::TsdfSubmapCollection::Ptr &submap_collection_ptr,
      const cblox::SubmapID &submap_id, const cblox::Color &submap_color,
      const std::string &submap_frame, const ros::Publisher &publisher) const;

 private:
  voxblox::MeshIntegratorConfig mesh_config_;
  cblox::TsdfSubmapMesher tsdf_submap_mesher_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_VISUALIZATION_SUBMAP_VISUALS_H_
