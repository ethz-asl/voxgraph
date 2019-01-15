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

  VoxgraphSubmap(const voxblox::Transformation &T_M_S,
                 cblox::SubmapID submap_id, Config config)
      : cblox::TsdfEsdfSubmap(T_M_S, submap_id, config) {}

  bool overlapsWith(VoxgraphSubmap::ConstPtr otherSubmap) const {
    // TODO(victorr): Implement collision checking and remove this dummy test
    return (std::abs(submap_id_ - otherSubmap->getID()) < 2);
  }

  // TODO(victorr): Move RelevantVoxelList from registration_cost_function here
};
}  // namespace voxgraph

#endif  // VOXGRAPH_VOXGRAPH_SUBMAP_H_
