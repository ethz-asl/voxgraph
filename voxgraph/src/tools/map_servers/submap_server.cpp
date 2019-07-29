#include "voxgraph/tools/map_servers/submap_server.h"
#include <minkindr_conversions/kindr_msg.h>

namespace voxgraph {
SubmapServer::SubmapServer(ros::NodeHandle nh_private) {
  tsdf_submap_pub_ =
      nh_private.advertise<voxgraph_msgs::Submap>("tsdf_submaps", 3, false);
  esdf_submap_pub_ =
      nh_private.advertise<voxgraph_msgs::Submap>("esdf_submaps", 3, false);
}

void SubmapServer::publishSubmapTsdf(const VoxgraphSubmap &submap) {
  voxgraph_msgs::Submap tsdf_submap_msg;
  tsdf_submap_msg.id = submap.getID();
  tf::poseKindrToMsg(submap.getPose().cast<double>(), &tsdf_submap_msg.pose);

  tsdf_submap_msg.layer_type = 0;
  voxblox::serializeLayerAsMsg<voxblox::TsdfVoxel>(
      submap.getTsdfMap().getTsdfLayer(), false, &tsdf_submap_msg.layer);
  tsdf_submap_msg.layer.action =
      static_cast<uint8_t>(voxblox::MapDerializationAction::kReset);
  tsdf_submap_pub_.publish(tsdf_submap_msg);
}

void SubmapServer::publishSubmapEsdf(const VoxgraphSubmap &submap) {
  voxgraph_msgs::Submap esdf_submap_msg;
  esdf_submap_msg.id = submap.getID();
  tf::poseKindrToMsg(submap.getPose().cast<double>(), &esdf_submap_msg.pose);

  esdf_submap_msg.layer_type = 1;
  voxblox::serializeLayerAsMsg<voxblox::EsdfVoxel>(
      submap.getEsdfMap().getEsdfLayer(), false, &esdf_submap_msg.layer);
  esdf_submap_msg.layer.action =
      static_cast<uint8_t>(voxblox::MapDerializationAction::kReset);
  esdf_submap_pub_.publish(esdf_submap_msg);
}
}  // namespace voxgraph
