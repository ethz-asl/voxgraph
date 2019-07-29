#ifndef VOXGRAPH_TOOLS_MAP_SERVERS_SUBMAP_SERVER_H_
#define VOXGRAPH_TOOLS_MAP_SERVERS_SUBMAP_SERVER_H_

#include <voxblox_ros/conversions.h>
#include <voxgraph_msgs/Submap.h>
#include "voxgraph/frontend/submap_collection/voxgraph_submap.h"

namespace voxgraph {
class SubmapServer {
 public:
  explicit SubmapServer(ros::NodeHandle nh_private);

  void publishSubmap(const VoxgraphSubmap &submap) {
    publishSubmapTsdf(submap);
    publishSubmapEsdf(submap);
  }

  void publishSubmapTsdf(const VoxgraphSubmap &submap);
  void publishSubmapEsdf(const VoxgraphSubmap &submap);

 private:
  ros::Publisher tsdf_submap_pub_;
  ros::Publisher esdf_submap_pub_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_TOOLS_MAP_SERVERS_SUBMAP_SERVER_H_
