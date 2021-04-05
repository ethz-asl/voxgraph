#ifndef VOXGRAPH_FRONTEND_FRAME_NAMES_H_
#define VOXGRAPH_FRONTEND_FRAME_NAMES_H_

#include <string>

#include <ros/ros.h>

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
  //   - T_O_B is provided by the odometry input
  //   - T_M_S corresponds the the optimized pose of the active submap
  //   - T_S_B corresponds to the pose of the robot in the current submap
  //   - T_B_C stores the transform from the pointcloud sensor
  //           to the base_link frame

  // Input frame names
  std::string input_odom_frame = "odom";
  std::string input_base_link_frame = "base_link";
  // NOTE: When 'use_tf_transforms' is set to True, the sensor_frame must also
  //       be defined. But we do not do this in this class since it is already
  //       given in the header.frame_id field of the PointCloud2 msgs.

  // Output frame names
  std::string output_mission_frame = "voxgraph_mission";
  std::string output_odom_frame = "voxgraph_odom";
  std::string output_active_submap_frame = "voxgraph_active_submap";
  std::string output_base_link_frame = "voxgraph_base_link";
  std::string output_sensor_frame = "voxgraph_sensor";

  // Static method to parse ROS params
  static FrameNames fromRosParams(const ros::NodeHandle& node_handle) {
    FrameNames frame_names;
    // Inputs
    node_handle.param("input_odom_frame", frame_names.input_odom_frame,
                      frame_names.input_odom_frame);
    node_handle.param("input_base_link_frame",
                      frame_names.input_base_link_frame,
                      frame_names.input_base_link_frame);

    // Outputs
    node_handle.param("output_mission_frame", frame_names.output_mission_frame,
                      frame_names.output_mission_frame);
    node_handle.param("output_odom_frame", frame_names.output_odom_frame,
                      frame_names.output_odom_frame);
    node_handle.param("output_active_submap_frame",
                      frame_names.output_active_submap_frame,
                      frame_names.output_active_submap_frame);
    node_handle.param("output_base_link_frame",
                      frame_names.output_base_link_frame,
                      frame_names.output_base_link_frame);
    node_handle.param("output_sensor_frame", frame_names.output_sensor_frame,
                      frame_names.output_sensor_frame);
    return frame_names;
  }
};
}  // namespace voxgraph

#endif  // VOXGRAPH_FRONTEND_FRAME_NAMES_H_
