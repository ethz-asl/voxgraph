#ifndef VOXGRAPH_TOOLS_SUBMAP_REPUBLISHER_H_
#define VOXGRAPH_TOOLS_SUBMAP_REPUBLISHER_H_

#include <string>
#include <unordered_set>

#include <ros/ros.h>
#include <transfolder_msgs/RobotSubfoldersArray.h>
#include <voxblox/core/common.h>
#include <voxblox/core/layer.h>
#include <voxblox/io/trajectory_io.h>

namespace voxgraph {
class SubmapRepublisher {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SubmapRepublisher(const ros::NodeHandle& nh,
                    const ros::NodeHandle& nh_private);

  void newSubmapNotificationCallback(
      const transfolder_msgs::RobotSubfoldersArray& msg);

  void publishSubmap(const voxblox::Layer<voxblox::TsdfVoxel>& tsdf_layer,
                     const voxblox::io::Trajectory& trajectory);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber new_submap_notification_sub_;
  ros::Publisher submap_pub_;

  std::unordered_set<std::string> processed_subfolder_paths_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_TOOLS_SUBMAP_REPUBLISHER_H_
