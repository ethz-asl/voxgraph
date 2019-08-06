#ifndef VOXGRAPH_FRONTEND_FRAME_NAMES_H_
#define VOXGRAPH_FRONTEND_FRAME_NAMES_H_

#include <ros/ros.h>
#include <string>

namespace voxgraph {
class FrameNames {
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

 public:
  FrameNames()
      :  // Default frame names
        mission_frame_("mission"),
        odom_frame_("odom"),
        base_link_frame_("base_link"),
        refined_frame_corrected_("refined_frame_corrected"),
        odom_frame_corrected_("odom_corrected"),
        base_link_frame_corrected_("base_link_corrected"),
        sensor_frame_corrected_("sensor_corrected"),
        // Route the public read-only accessors
        mission_frame(mission_frame_),
        odom_frame(odom_frame_),
        base_link_frame(base_link_frame_),
        refined_frame_corrected(refined_frame_corrected_),
        odom_frame_corrected(odom_frame_corrected_),
        base_link_frame_corrected(base_link_frame_corrected_),
        sensor_frame_corrected(sensor_frame_corrected_) {}

  // Input frame names
  // Public read-only accessors
  const std::string &mission_frame;
  const std::string &odom_frame;
  const std::string &base_link_frame;
  // NOTE: When 'use_tf_transforms' is set to True, the sensor_frame must also
  //       be defined. But we do not do this in this class since it is already
  //       given in the header.frame_id field of the PointCloud2 msgs.

  // Output frame names
  // Public read-only accessors
  const std::string &refined_frame_corrected;
  const std::string &odom_frame_corrected;
  const std::string &base_link_frame_corrected;
  const std::string &sensor_frame_corrected;

  // Static method to parse ROS params
  static FrameNames fromRosParams(const ros::NodeHandle &node_handle) {
    FrameNames frame_names;
    node_handle.param("mission_frame", frame_names.mission_frame_,
                      frame_names.mission_frame_);
    node_handle.param("odom_frame", frame_names.odom_frame_,
                      frame_names.odom_frame_);
    node_handle.param("base_link_frame", frame_names.base_link_frame_,
                      frame_names.base_link_frame_);
    node_handle.param("refined_frame_corrected",
                      frame_names.refined_frame_corrected_,
                      frame_names.refined_frame_corrected_);
    node_handle.param("odom_frame_corrected", frame_names.odom_frame_corrected_,
                      frame_names.odom_frame_corrected_);
    node_handle.param("base_link_frame_corrected",
                      frame_names.base_link_frame_corrected_,
                      frame_names.base_link_frame_corrected_);
    node_handle.param("sensor_frame_corrected",
                      frame_names.sensor_frame_corrected_,
                      frame_names.sensor_frame_corrected_);
    return frame_names;
  }

 private:
  // Input frame names
  std::string mission_frame_;
  std::string odom_frame_;
  std::string base_link_frame_;

  // Output frame names
  std::string refined_frame_corrected_;
  std::string odom_frame_corrected_;
  std::string base_link_frame_corrected_;
  std::string sensor_frame_corrected_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_FRONTEND_FRAME_NAMES_H_
