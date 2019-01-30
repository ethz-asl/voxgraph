//
// Created by victor on 29.01.19.
//

#include "voxgraph/odometry_simulator/odometry_simulator.h"
#include <eigen_conversions/eigen_msg.h>
#include <kindr/minimal/rotation-quaternion.h>
#include <minkindr_conversions/kindr_msg.h>
#include <tf2_ros/transform_broadcaster.h>

namespace voxgraph {
OdometrySimulator::OdometrySimulator(const ros::NodeHandle &nh,
                                     const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private), subscriber_queue_length_(100) {
  // Read odometry simulator params from ROS
  nh_private_.param("subscriber_queue_length_", subscriber_queue_length_,
                    subscriber_queue_length_);
  nh_private_.param<double>("x_mean", noise_.x.mean(), 0);
  nh_private_.param<double>("x_stddev", noise_.x.stddev(), 0);
  nh_private_.param<double>("y_mean", noise_.y.mean(), 0);
  nh_private_.param<double>("y_stddev", noise_.y.stddev(), 0);
  nh_private_.param<double>("z_mean", noise_.z.mean(), 0);
  nh_private_.param<double>("z_stddev", noise_.z.stddev(), 0);
  nh_private_.param<double>("yaw_mean", noise_.yaw.mean(), 0);
  nh_private_.param<double>("yaw_stddev", noise_.yaw.stddev(), 0);
  nh_private_.param<double>("pitch_mean", noise_.pitch.mean(), 0);
  nh_private_.param<double>("pitch_stddev", noise_.pitch.stddev(), 0);
  nh_private_.param<double>("roll_mean", noise_.roll.mean(), 0);
  nh_private_.param<double>("roll_stddev", noise_.roll.stddev(), 0);
  nh_private_.param<double>("x_vel_mean", noise_.x_vel.mean(), 0);
  nh_private_.param<double>("x_vel_stddev", noise_.x_vel.stddev(), 0);
  nh_private_.param<double>("y_vel_mean", noise_.y_vel.mean(), 0);
  nh_private_.param<double>("y_vel_stddev", noise_.y_vel.stddev(), 0);
  nh_private_.param<double>("z_vel_mean", noise_.z_vel.mean(), 0);
  nh_private_.param<double>("z_vel_stddev", noise_.z_vel.stddev(), 0);
  nh_private_.param<double>("yaw_rate_mean", noise_.yaw_rate.mean(), 0);
  nh_private_.param<double>("yaw_rate_stddev", noise_.yaw_rate.stddev(), 0);

  // Subscribe to the odometry ROS topic
  odometry_subscriber_ =
      nh_.subscribe("odometry", subscriber_queue_length_,
                    &OdometrySimulator::odometryCallback, this);
}

void OdometrySimulator::odometryCallback(
    const nav_msgs::Odometry::ConstPtr &odometry_msg) {
  typedef kindr::minimal::RotationQuaternionTemplate<double> Rotation;
  typedef Eigen::Matrix<double, 3, 1> Vector3;
  // Check if this is the first odometry message
  if (pose_stamped_.header.stamp.isZero()) {
    // Initialize the stored pose
    pose_stamped_.header = odometry_msg->header;
    pose_stamped_.child_frame_id = odometry_msg->child_frame_id;
    pose_stamped_.transform.rotation = odometry_msg->pose.pose.orientation;
    ROS_INFO("Initialized drifting odometry simulator");
    publishCurrentPoseTf();
    return;
  }
  // Calculate time delta since last message
  double Dt = (odometry_msg->header.stamp - pose_stamped_.header.stamp).toSec();

  // Load quantities of interest
  // Body position in world frame
  Vector3 W_translation_WB;
  tf::vectorMsgToKindr(pose_stamped_.transform.translation, &W_translation_WB);
  // Body orientation in the world frame
  Rotation Rotation_WB;
  tf::quaternionMsgToKindr(pose_stamped_.transform.rotation, &Rotation_WB);
  // Linear velocity in body frame
  Eigen::Vector3d B_linear_velocity;
  tf::vectorMsgToEigen(odometry_msg->twist.twist.linear, B_linear_velocity);
  // Angular velocity in body frame
  Eigen::Vector3d B_angular_velocity;
  tf::vectorMsgToEigen(odometry_msg->twist.twist.angular, B_angular_velocity);

  // Update the position
  // NOTE: This simulation does not account for correlated noise
  // Simulate noise on measured linear velocities
  B_linear_velocity.x() += noise_.x_vel();
  B_linear_velocity.y() += noise_.y_vel();
  B_linear_velocity.z() += noise_.z_vel();
  // Transform linear velocity into world frame
  Eigen::Vector3d W_linear_velocity =
      Rotation_WB.getRotationMatrix() * B_linear_velocity;
  // Integrate translation
  W_translation_WB.x() += W_linear_velocity.x() * Dt;
  W_translation_WB.y() += W_linear_velocity.y() * Dt;
  W_translation_WB.z() += W_linear_velocity.z() * Dt;

  // Update the orientation
  // NOTE: We assume that a good gravity estimate is available,
  //       to eliminate pitch and roll drift
  // Simulate noise on yaw rate
  B_angular_velocity.x() += noise_.yaw_rate();
  // Transform angular velocity into world frame
  Eigen::Vector3d W_angular_velocity =
      Rotation_WB.getRotationMatrix() * B_angular_velocity;
  // Integrate angular velocity
  Eigen::Vector3d rotation_vector = W_angular_velocity * Dt;
  Rotation Rotation_WB_Delta(rotation_vector);
  Rotation_WB = Rotation_WB_Delta * Rotation_WB;
  Rotation_WB.normalize();

  // Update stored pose
  pose_stamped_.header = odometry_msg->header;
  tf::quaternionKindrToMsg(Rotation_WB, &pose_stamped_.transform.rotation);
  tf::vectorKindrToMsg(W_translation_WB, &pose_stamped_.transform.translation);
  publishCurrentPoseTf();
}

void OdometrySimulator::publishCurrentPoseTf() {
  static tf2_ros::TransformBroadcaster transform_broadcaster;
  transform_broadcaster.sendTransform(pose_stamped_);
}
}  // namespace voxgraph
