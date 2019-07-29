#ifndef VOXGRAPH_TOOLS_MAP_SERVERS_PROJECTED_MAP_SERVER_H_
#define VOXGRAPH_TOOLS_MAP_SERVERS_PROJECTED_MAP_SERVER_H_

#include <voxblox_msgs/Layer.h>
#include <voxblox_ros/conversions.h>
#include "voxgraph/frontend/submap_collection/voxgraph_submap_collection.h"

namespace voxgraph {
class ProjectedMapServer {
 public:
  explicit ProjectedMapServer(ros::NodeHandle nh_private);

  void publishProjectedMap(const VoxgraphSubmapCollection &submap_collection);

 private:
  ros::Publisher projected_tsdf_map_pub_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_TOOLS_MAP_SERVERS_PROJECTED_MAP_SERVER_H_
