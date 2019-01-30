//
// Created by victor on 29.01.19.
//

#ifndef VOXGRAPH_ODOMETRY_SIMULATOR_ODOMETRY_SIMULATOR_H_
#define VOXGRAPH_ODOMETRY_SIMULATOR_ODOMETRY_SIMULATOR_H_

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include "voxgraph/odometry_simulator/normal_distribution.h"

namespace voxgraph {
class OdometrySimulator {
 public:
  explicit OdometrySimulator(const ros::NodeHandle &nh,
                             const ros::NodeHandle &nh_private);
  ~OdometrySimulator() = default;

  void odometryCallback(const nav_msgs::Odometry::ConstPtr &odometry_msg);

 private:
  // Internal pose estimate
  // NOTE: TransformStamped is used instead of PoseStamped to avoid an
  //       unnecessary conversion step when publishing the current pose
  //       TF and as a convenient way to store the child_frame_id
  geometry_msgs::TransformStamped pose_stamped_;

  // ROS Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Odometry subscriber and related settings
  ros::Subscriber odometry_subscriber_;
  int subscriber_queue_length_;

  // Noise distributions
  struct NoiseDistributions {
    NormalDistribution x, y, z;
    NormalDistribution yaw, pitch, roll;
    NormalDistribution x_vel, y_vel, z_vel;
    NormalDistribution yaw_rate;
  } noise_;

  // Transform publisher
  void publishCurrentPoseTf();
};
}  // namespace voxgraph

#endif  // VOXGRAPH_ODOMETRY_SIMULATOR_ODOMETRY_SIMULATOR_H_
