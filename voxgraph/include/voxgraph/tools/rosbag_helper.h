#ifndef VOXGRAPH_TOOLS_ROSBAG_HELPER_H_
#define VOXGRAPH_TOOLS_ROSBAG_HELPER_H_

#include <ros/ros.h>
#include <std_srvs/SetBool.h>

namespace voxgraph {
class RosbagHelper {
 public:
  explicit RosbagHelper(ros::NodeHandle node_handle);

  void pauseRosbag();
  void playRosbag();

 private:
  ros::ServiceClient rosbag_pause_srv_;
  std_srvs::SetBool srv_msg_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_TOOLS_ROSBAG_HELPER_H_
