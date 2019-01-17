//
// Created by victor on 08.01.19.
//

#ifndef VOXGRAPH_VOXGRAPH_SUBMAP_H_
#define VOXGRAPH_VOXGRAPH_SUBMAP_H_

#include <cblox/core/tsdf_esdf_submap.h>
#include <memory>

namespace voxgraph {
class VoxgraphSubmap : public cblox::TsdfEsdfSubmap {
 public:
  typedef std::shared_ptr<VoxgraphSubmap> Ptr;
  typedef std::shared_ptr<const VoxgraphSubmap> ConstPtr;

  struct BoundingBox {
    voxblox::Point min = {0, 0, 0};
    voxblox::Point max = {0, 0, 0};
  };

  VoxgraphSubmap(const voxblox::Transformation &T_M_S,
                 cblox::SubmapID submap_id, Config config)
      : cblox::TsdfEsdfSubmap(T_M_S, submap_id, config) {}

  const BoundingBox getSurfaceObb() const;
  const BoundingBox getMapObb() const;
  const BoundingBox getSurfaceAabb() const {}
  const BoundingBox getMapAabb() const {}

  bool overlapsWith(VoxgraphSubmap::ConstPtr otherSubmap) const {
    // TODO(victorr): Implement collision checking
    //                and remove this dummy 'proximity' test
    unsigned int other_submap_id = otherSubmap->getID();
    if (submap_id_ > other_submap_id) {
      return (submap_id_ - other_submap_id) < 2;
    } else {
      return (other_submap_id - submap_id_) < 2;
    }
  }

  // TODO(victorr): Move RelevantVoxelList from registration_cost_function here

 private:
  // Oriented Bounding Boxes in submap frame
  mutable BoundingBox surface_obb_;  // around the isosurface
  mutable BoundingBox map_obb_;      // around the entire submap
  // Axis Aligned Bounding Boxes in world frame
  mutable BoundingBox surface_aabb_;  // around the isosurface
  mutable BoundingBox map_aabb_;      // around the entire submap
};
}  // namespace voxgraph

#endif  // VOXGRAPH_VOXGRAPH_SUBMAP_H_
