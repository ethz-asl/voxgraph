#ifndef VOXGRAPH_TOOLS_ROS_PARAMS_H_
#define VOXGRAPH_TOOLS_ROS_PARAMS_H_

#include <ros/node_handle.h>
#include <voxblox_ros/ros_params.h>

#include "voxgraph/frontend/submap_collection/voxgraph_submap.h"

namespace voxgraph {
inline VoxgraphSubmap::Config getVoxgraphSubmapConfigFromRosParams(
    const ros::NodeHandle& nh_private) {
  // Getting the tsdf map params from ROS.
  const voxblox::TsdfMap::Config tsdf_map_config =
      voxblox::getTsdfMapConfigFromRosParam(nh_private);
  const voxblox::EsdfMap::Config esdf_map_config =
      voxblox::getEsdfMapConfigFromRosParam(nh_private);

  // Copying over the members
  // Note(alexmillane): This is very prone to failure down the line.
  // The basic problem is that inheritance is the wrong thing to use for these
  // config structs.
  VoxgraphSubmap::Config config;
  config.tsdf_voxel_size = tsdf_map_config.tsdf_voxel_size;
  config.tsdf_voxels_per_side = tsdf_map_config.tsdf_voxels_per_side;
  config.esdf_voxel_size = esdf_map_config.esdf_voxel_size;
  config.esdf_voxels_per_side = esdf_map_config.esdf_voxels_per_side;

  // TODO(alexmillane): RegistrationFilter params are not grabbed yet here

  return config;
}
}  // namespace voxgraph

#endif  // VOXGRAPH_TOOLS_ROS_PARAMS_H_
