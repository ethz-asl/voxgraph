#ifndef VOXGRAPH_TOOLS_MAP_SERVERS_SUBMAP_SERVER_H_
#define VOXGRAPH_TOOLS_MAP_SERVERS_SUBMAP_SERVER_H_

#include <std_msgs/Header.h>
#include <voxgraph_msgs/MapHeader.h>
#include "voxgraph/frontend/submap_collection/voxgraph_submap.h"

namespace voxgraph {
class SubmapServer {
 public:
  explicit SubmapServer(ros::NodeHandle nh_private);

  void publishSubmap(const VoxgraphSubmap &submap, const ros::Time &timestamp);
  void publishSubmapTsdf(const VoxgraphSubmap &submap,
                         const ros::Time &timestamp);
  void publishSubmapEsdf(const VoxgraphSubmap &submap,
                         const ros::Time &timestamp);
  void publishSubmapSurfacePointcloud(const VoxgraphSubmap &submap,
                                      const ros::Time &timestamp);

 private:
  ros::Publisher submap_tsdf_pub_;
  ros::Publisher submap_esdf_pub_;
  ros::Publisher submap_surface_pointcloud_pub_;

  std_msgs::Header generateHeaderMsg(const VoxgraphSubmap &submap,
                                     const ros::Time &timestamp);
  voxgraph_msgs::MapHeader generateSubmapHeaderMsg(
      const VoxgraphSubmap &submap);
};
}  // namespace voxgraph

#endif  // VOXGRAPH_TOOLS_MAP_SERVERS_SUBMAP_SERVER_H_
