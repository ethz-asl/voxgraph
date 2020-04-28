#include "voxgraph/tools/data_servers/submap_conversions.h"

namespace cblox {

template <>
cblox_msgs::MapHeader generateSubmapHeaderMsg<voxgraph::VoxgraphSubmap>(
    const voxgraph::VoxgraphSubmap::Ptr& submap_ptr) {
  cblox_msgs::MapHeader submap_header =
      cblox::generateSubmapHeaderMsg<TsdfEsdfSubmap>(submap_ptr);

  // Set the submap's start and end time
  const voxgraph::VoxgraphSubmap::PoseHistoryMap& pose_history =
      submap_ptr->getPoseHistory();
  if (!pose_history.empty()) {
    submap_header.start_time = pose_history.begin()->first;
    submap_header.end_time = (--pose_history.end())->first;
  } else {
    submap_header.start_time = ros::Time(0.0);
    submap_header.end_time = ros::Time(0.0);
  }

  // TODO(victorr): Get the world frame name from FrameNames once implemented
  submap_header.pose_estimate.frame_id = "mission";

  return submap_header;
}

}