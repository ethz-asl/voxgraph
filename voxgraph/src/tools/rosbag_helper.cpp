#include "voxgraph/tools/rosbag_helper.h"

namespace voxgraph {
RosbagHelper::RosbagHelper(ros::NodeHandle node_handle) {
  rosbag_pause_srv_ =
      node_handle.serviceClient<std_srvs::SetBool>("/player/pause_playback");
}

void RosbagHelper::pauseRosbag() {
  srv_msg_.request.data = true;
  rosbag_pause_srv_.call(srv_msg_);
}

void RosbagHelper::playRosbag() {
  srv_msg_.request.data = false;
  rosbag_pause_srv_.call(srv_msg_);
}
}  // namespace voxgraph
