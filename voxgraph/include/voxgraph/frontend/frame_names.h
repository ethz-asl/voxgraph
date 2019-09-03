#ifndef VOXGRAPH_FRONTEND_FRAME_NAMES_H_
#define VOXGRAPH_FRONTEND_FRAME_NAMES_H_

#include <ros/ros.h>
#include <string>

namespace voxgraph {
struct FrameNames {
  // Coordinate frame naming convention
  //   - M: Mission (for a single run of a single robot this corresponds to the
  //   World frame)
  //   - O: Odometry input frame
  //   - B: Base link (often corresponds to the robot's IMU frame)
  //   - C: Sensor frame of the pointcloud sensor
  //   - S: Active submap (used by the PointcloudIntegrator and MapTracker)
  //  The full transform input and output chains are O -> B and M -> S -> B -> C
  //  respectively, where:
  //   - T_M_O aggregates the incremental pose corrections from ICP
  //   - T_O_B is provided by the odometry input
  //   - T_M_S corresponds the the optimized pose of the active submap
  //   - T_S_B corresponds to the pose of the robot in the current submap
  //   - T_B_C stores the transform from the pointcloud sensor
  //           to the base_link frame
  // TODO(victorr): Update this documentation once Voxgraph's ICP has been
  //                reimplemented to work with the new frame convention

  // Input frame names
  std::string mission_frame = "mission";
  std::string odom_frame = "odom";
  std::string base_link_frame = "base_link";
  // NOTE: When 'use_tf_transforms' is set to True, the sensor_frame must also
  //       be defined. But we do not do this in this class since it is already
  //       given in the header.frame_id field of the PointCloud2 msgs.

  // Output frame names
  std::string odom_frame_corrected = "odom_corrected";
  std::string active_submap_frame = "active_submap";
  std::string base_link_frame_corrected = "base_link_corrected";
  std::string sensor_frame_corrected = "sensor_corrected";

  // Static method to parse ROS params
  static FrameNames fromRosParams(const ros::NodeHandle &node_handle) {
    FrameNames frame_names;
    node_handle.param("mission_frame", frame_names.mission_frame,
                      frame_names.mission_frame);
    node_handle.param("odom_frame", frame_names.odom_frame,
                      frame_names.odom_frame);
    node_handle.param("base_link_frame", frame_names.base_link_frame,
                      frame_names.base_link_frame);
    node_handle.param("active_submap_frame", frame_names.active_submap_frame,
                      frame_names.active_submap_frame);
    node_handle.param("odom_frame_corrected", frame_names.odom_frame_corrected,
                      frame_names.odom_frame_corrected);
    node_handle.param("base_link_frame_corrected",
                      frame_names.base_link_frame_corrected,
                      frame_names.base_link_frame_corrected);
    node_handle.param("sensor_frame_corrected",
                      frame_names.sensor_frame_corrected,
                      frame_names.sensor_frame_corrected);
    return frame_names;
  }
};
}  // namespace voxgraph

#endif  // VOXGRAPH_FRONTEND_FRAME_NAMES_H_
