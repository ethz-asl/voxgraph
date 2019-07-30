#ifndef VOXGRAPH_TOOLS_MAP_SERVERS_PROJECTED_MAP_SERVER_H_
#define VOXGRAPH_TOOLS_MAP_SERVERS_PROJECTED_MAP_SERVER_H_

#include <voxgraph_msgs/MapHeader.h>
#include "voxgraph/common.h"
#include "voxgraph/frontend/submap_collection/voxgraph_submap_collection.h"

namespace voxgraph {
class ProjectedMapServer {
 public:
  explicit ProjectedMapServer(ros::NodeHandle nh_private);

  void publishProjectedMap(const VoxgraphSubmapCollection &submap_collection,
                           const ros::Time &timestamp);

 private:
  ros::Publisher projected_tsdf_map_pub_;

  std_msgs::Header generateHeaderMsg(const ros::Time &timestamp);

  voxgraph_msgs::MapHeader generateMapHeaderMsg(
      const VoxgraphSubmapCollection &submap_collection);
};
}  // namespace voxgraph

#endif  // VOXGRAPH_TOOLS_MAP_SERVERS_PROJECTED_MAP_SERVER_H_
