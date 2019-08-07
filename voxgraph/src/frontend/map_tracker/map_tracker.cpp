#include "voxgraph/frontend/map_tracker/map_tracker.h"
#include "voxgraph/tools/tf_helper.h"

namespace voxgraph {
void MapTracker::registerPointcloud(
    const sensor_msgs::PointCloud2::Ptr &pointcloud_msg) {
  Transformation T_world_sensor_refined;
  bool pose_refinement_successful = scan_to_map_registerer_.refineSensorPose(
      pointcloud_msg, T_world_sensor, &T_world_sensor_refined);
  if (pose_refinement_successful) {
    // Update the robot and sensor pose
    T_world_sensor = T_world_sensor_refined;
    T_world_robot = T_world_sensor * T_robot_sensor_.inverse();
    // Update and publish the corrected odometry frame
    T_world_odom_corrected_ = T_world_robot * T_odom_robot.inverse();
    TfHelper::publishTransform(T_world_odom_corrected_, world_frame_,
                               odom_frame_corrected_, true, current_timestamp);
  } else {
    ROS_WARN("Pose refinement failed");
  }
}

bool MapTracker::lookup_T_odom_base_link(ros::Time timestamp,
                                         Transformation *T_odom_base) {
  CHECK_NOTNULL(T_odom_base);
  Transformation T_odom_base_received;
  double t_waited = 0;  // Total time spent waiting for the updated pose
  double t_max = 0.20;  // Maximum time to wait before giving up
  const ros::Duration timeout(0.005);  // Timeout between each update attempt
  while (t_waited < t_max) {
    if (transformer_.lookupTransform(frame_names_.base_link_frame,
                                     frame_names_.odom_frame, timestamp,
                                     &T_odom_base_received)) {
      *T_odom_base = T_odom_base_received;
      return true;
    }
    timeout.sleep();
    t_waited += timeout.toSec();
  }
  ROS_WARN("Waited %.3fs, but still could not get the TF from %s to %s",
           t_waited, frame_names_.base_link_frame.c_str(),
           frame_names_.odom_frame.c_str());
  return false;
}

void MapTracker::publishTFs(const ros::Time &timestamp) {
  TfHelper::publishTransform(T_M_L_, frame_names_.mission_frame,
                             frame_names_.refined_frame_corrected, false,
                             timestamp);
  TfHelper::publishTransform(T_L_O_, frame_names_.refined_frame_corrected,
                             frame_names_.odom_frame_corrected, false,
                             timestamp);
  TfHelper::publishTransform(T_odom_base, frame_names_.odom_frame_corrected,
                             frame_names_.base_link_frame_corrected, false,
                             timestamp);
  TfHelper::publishTransform(T_B_C_, frame_names_.base_link_frame_corrected,
                             frame_names_.sensor_frame_corrected, true,
                             timestamp);
}
}  // namespace voxgraph
