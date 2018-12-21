//
// Created by victor on 19.12.18.
//

#ifndef VOXGRAPH_VISUALIZATION_H
#define VOXGRAPH_VISUALIZATION_H

#include <ros/ros.h>
#include <minkindr_conversions/kindr_tf.h>
#include <cblox/core/tsdf_submap.h>
#include <cblox/mesh/tsdf_submap_mesher.h>

namespace voxgraph {
  class Visualization {
  public:
    explicit Visualization(const cblox::TsdfMap::Config& tsdf_map_config);

    void publishMesh(cblox::TsdfSubmapCollection::Ptr &submap_collection_ptr,
                     const cblox::SubmapID &submap_id,
                     const cblox::Color &submap_color,
                     const std::string &submap_frame,
                     const ros::Publisher& publisher) const;

    void publishTransform(const voxblox::Transformation &transform,
                          const std::string &base_frame,
                          const std::string &target_frame,
                          bool tf_is_static=false) const;

  private:
    voxblox::MeshIntegratorConfig mesh_config_;
    cblox::TsdfSubmapMesher tsdf_submap_mesher_;
  };
}


#endif //VOXGRAPH_VISUALIZATION_H
