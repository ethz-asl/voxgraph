#ifndef VOXGRAPH_FRONTEND_FRAME_NAMES_H_
#define VOXGRAPH_FRONTEND_FRAME_NAMES_H_

#include <ros/ros.h>
#include <string>

namespace voxgraph {
struct FrameNames {
  // Coordinate frame naming convention
  //   - M: Mission (for a single run of a single robot this corresponds to the
  //   World frame)
  //   - L: Fictive frame used to track the loop closure and
  //        scan-to-map-registration corrections
  //   - O: Odometry input frame
  //   - B: Base link (often corresponds to the robot's IMU frame)
  //   - C: Sensor frame of the pointcloud sensor
  //   - S: Active submap
  //  The full transform chain is M -> L -> O -> B -> C, where:
  //   - T_M_L aggregates the pose corrections from loop closures
  //   - T_L_O aggregates the incremental pose corrections from ICP
  //   - T_O_B is provided by the odometry input
  //   - T_B_C corresponds to the extrinsic calibration

  // Input frame names
  std::string mission_frame = "mission";
  std::string odom_frame = "odom";
  std::string base_link_frame = "base_link";
  // NOTE: When 'use_tf_transforms' is set to True, the sensor_frame must also
  //       be defined. But we do not do this in this class since it is already
  //       given in the header.frame_id field of the PointCloud2 msgs.

  // Output frame names
  std::string refined_frame_corrected = "refined_frame_corrected";
  std::string odom_frame_corrected = "odom_corrected";
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
    node_handle.param("refined_frame_corrected",
                      frame_names.refined_frame_corrected,
                      frame_names.refined_frame_corrected);
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
