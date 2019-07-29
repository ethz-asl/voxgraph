#include "voxgraph/tools/map_servers/projected_map_server.h"

namespace voxgraph {
ProjectedMapServer::ProjectedMapServer(ros::NodeHandle nh_private) {
  projected_tsdf_map_pub_ =
      nh_private.advertise<voxblox_msgs::Layer>("projected_map", 1, true);
}

void ProjectedMapServer::publishProjectedMap(
    const VoxgraphSubmapCollection &submap_collection) {
  voxblox_msgs::Layer projected_tsdf_layer_msg;
  voxblox::serializeLayerAsMsg<voxblox::TsdfVoxel>(
      submap_collection.getProjectedMap()->getTsdfLayer(), false,
      &projected_tsdf_layer_msg);
  projected_tsdf_layer_msg.action =
      static_cast<uint8_t>(voxblox::MapDerializationAction::kReset);
  projected_tsdf_map_pub_.publish(projected_tsdf_layer_msg);
}
}  // namespace voxgraph
