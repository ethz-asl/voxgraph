#ifndef VOXGRAPH_INCLUDE_VOXGRAPH_TOOLS_DATA_SERVERS_SUBMAP_CONVERSIONS_H_
#define VOXGRAPH_INCLUDE_VOXGRAPH_TOOLS_DATA_SERVERS_SUBMAP_CONVERSIONS_H_

#include <cblox_ros/submap_conversions.h>
#include <voxgraph/frontend/submap_collection/voxgraph_submap.h>

namespace cblox {

template <>
cblox_msgs::MapHeader generateSubmapHeaderMsg<voxgraph::VoxgraphSubmap>(
    const voxgraph::VoxgraphSubmap::Ptr& submap_ptr);

}

#endif //VOXGRAPH_INCLUDE_VOXGRAPH_TOOLS_DATA_SERVERS_SUBMAP_CONVERSIONS_H_
