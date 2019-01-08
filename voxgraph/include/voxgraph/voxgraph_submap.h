//
// Created by victor on 08.01.19.
//

#ifndef VOXGRAPH_VOXGRAPH_SUBMAP_H_
#define VOXGRAPH_VOXGRAPH_SUBMAP_H_

#include <cblox/core/tsdf_esdf_submap.h>

namespace voxgraph {
typedef cblox::TsdfEsdfSubmap VoxgraphSubmap;
// TODO(victorr): Create VoxgraphSubmap child class containing the
//                RelevantVoxelList from registration_cost_function
// class VoxgraphSubmap : public cblox::TsdfEsdfSubmap {};
}  // namespace voxgraph

#endif  // VOXGRAPH_VOXGRAPH_SUBMAP_H_
