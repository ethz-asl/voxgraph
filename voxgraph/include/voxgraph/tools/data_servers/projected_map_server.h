#ifndef VOXGRAPH_TOOLS_DATA_SERVERS_PROJECTED_MAP_SERVER_H_
#define VOXGRAPH_TOOLS_DATA_SERVERS_PROJECTED_MAP_SERVER_H_

#include <cblox_msgs/MapHeader.h>

#include "voxgraph/common.h"
#include "voxgraph/frontend/submap_collection/voxgraph_submap_collection.h"

namespace voxgraph {
class ProjectedMapServer {
 public:
  explicit ProjectedMapServer(ros::NodeHandle nh_private);

  // Publish the map using this map server instance's ros publisher member
  void publishProjectedMap(const VoxgraphSubmapCollection& submap_collection,
                           const ros::Time& timestamp);

  // "Bring your own publisher" method
  // NOTE: This method is provided s.t. it can be called using publishers to
  //       custom topics and without requiring a ProjectedMapServer instance.
  //       It is therefore static.
  static void publishProjectedMap(
      const VoxgraphSubmapCollection& submap_collection,
      const ros::Time& timestamp,
      const ros::Publisher& projected_map_publisher);

 private:
  ros::Publisher projected_tsdf_map_pub_;

  // Convenience methods to generate the message and submap headers
  static std_msgs::Header generateHeaderMsg(const ros::Time& timestamp);
  static cblox_msgs::MapHeader generateMapHeaderMsg(
      const VoxgraphSubmapCollection& submap_collection);
};
}  // namespace voxgraph

#endif  // VOXGRAPH_TOOLS_DATA_SERVERS_PROJECTED_MAP_SERVER_H_
