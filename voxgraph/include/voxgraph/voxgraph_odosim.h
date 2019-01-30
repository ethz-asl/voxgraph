//
// Created by victor on 29.01.19.
//

#ifndef VOXGRAPH_VOXGRAPH_ODOSIM_H
#define VOXGRAPH_VOXGRAPH_ODOSIM_H

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

namespace voxgraph {
class VoxgraphOdosim {
 public:
  VoxgraphOdosim();
  ~VoxgraphOdosim();

  void odometryCallback(const nav_msgs::Odometry::ConstPtr &odometry_msg);

 private:
  ros::Subscriber odometry_subscriber_;

  // Imperfect odometry simulator
  geometry_msgs::PoseStamped odom_simulator_pose_stamped_;
  double odom_simulator_noise_linear_vel_mean_ = 0;
  double odom_simulator_noise_linear_vel_stddev_ = 0;
  double odom_simulator_noise_angular_vel_mean_ = 0;
  double odom_simulator_noise_angular_vel_stddev_ = 0;
  std::string odom_base_frame_ = "odom";
  std::default_random_engine odom_simulator_noise_generator_;
  std::normal_distribution<double> odom_simulator_noise_linear_vel_dist_;
  std::normal_distribution<double> odom_simulator_noise_angular_vel_dist_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_VOXGRAPH_ODOSIM_H
