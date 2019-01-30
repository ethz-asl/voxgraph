//
// Created by victor on 29.01.19.
//

#include "voxgraph/voxgraph_odosim.h"

namespace voxgraph {
VoxgraphOdosim::VoxgraphOdosim()
    : odom_simulator_noise_linear_vel_dist_(
          odom_simulator_noise_linear_vel_mean_,
          odom_simulator_noise_linear_vel_stddev_),
      odom_simulator_noise_angular_vel_dist_(
          odom_simulator_noise_angular_vel_mean_,
          odom_simulator_noise_angular_vel_stddev_) {
  // Read odometry simulator params from ROS
  nh_private_.param("odom_base_frame", odom_base_frame_, odom_base_frame_);
  nh_private_.param("odom_simulator_noise_linear_vel_mean",
                    odom_simulator_noise_linear_vel_mean_,
                    odom_simulator_noise_linear_vel_mean_);
  nh_private_.param("odom_simulator_noise_linear_vel_stddev",
                    odom_simulator_noise_linear_vel_stddev_,
                    odom_simulator_noise_linear_vel_stddev_);
  odom_simulator_noise_linear_vel_dist_ =
      std::normal_distribution<double>(odom_simulator_noise_linear_vel_mean_,
                                       odom_simulator_noise_linear_vel_stddev_);
  nh_private_.param("odom_simulator_noise_angular_vel_mean",
                    odom_simulator_noise_angular_vel_mean_,
                    odom_simulator_noise_angular_vel_mean_);
  nh_private_.param("odom_simulator_noise_angular_vel_stddev",
                    odom_simulator_noise_angular_vel_stddev_,
                    odom_simulator_noise_angular_vel_stddev_);
  odom_simulator_noise_angular_vel_dist_ = std::normal_distribution<double>(
      odom_simulator_noise_angular_vel_mean_,
      odom_simulator_noise_angular_vel_stddev_);

  odometry_subscriber_ = nh_.subscribe("odometry", subscriber_queue_length_,
                                       &VoxgraphMapper::odometryCallback, this);
}

void VoxgraphOdosim::odometryCallback(
    const nav_msgs::Odometry::ConstPtr &odometry_msg) {
  // If this is the first odometry msg,
  // we do not know Dt yet and postpone integration to the next msg
  if (odom_simulator_pose_stamped_.header.stamp.isZero()) {
    ROS_INFO("Initialized drifting odometry simulator");
    odom_simulator_pose_stamped_.header.stamp = odometry_msg->header.stamp;
    // Initialize pose (note: important that orientation is correct)
    odom_simulator_pose_stamped_.pose = odometry_msg->pose.pose;
    return;
  }
  // Calculate time delta since last message
  double Dt =
      (odometry_msg->header.stamp - odom_simulator_pose_stamped_.header.stamp)
          .toSec();

  // Load quantities of interest
  Eigen::Vector3d position;  // Previous pose position in world frame
  tf::pointMsgToEigen(odom_simulator_pose_stamped_.pose.position, position);
  Eigen::Quaterniond orientation;  // Previous pose orientation in world frame
  tf::quaternionMsgToEigen(odom_simulator_pose_stamped_.pose.orientation,
                           orientation);
  Eigen::Vector3d linear_velocity__bodyframe;
  tf::vectorMsgToEigen(odometry_msg->twist.twist.linear,
                       linear_velocity__bodyframe);
  Eigen::Vector3d angular_velocity__bodyframe;
  tf::vectorMsgToEigen(odometry_msg->twist.twist.angular,
                       angular_velocity__bodyframe);

  // Simulate noise on measured velocities
  // TODO(victorr): Sample the noise from a multivariate gaussian
  //  (instead of assuming x,y,z are uncorrelated)
  linear_velocity__bodyframe.x() +=
      odom_simulator_noise_linear_vel_dist_(odom_simulator_noise_generator_);
  linear_velocity__bodyframe.y() +=
      odom_simulator_noise_linear_vel_dist_(odom_simulator_noise_generator_);
  linear_velocity__bodyframe.z() +=
      odom_simulator_noise_linear_vel_dist_(odom_simulator_noise_generator_);
  angular_velocity__bodyframe.x() +=
      odom_simulator_noise_angular_vel_dist_(odom_simulator_noise_generator_);
  angular_velocity__bodyframe.y() +=
      odom_simulator_noise_angular_vel_dist_(odom_simulator_noise_generator_);
  angular_velocity__bodyframe.z() +=
      odom_simulator_noise_angular_vel_dist_(odom_simulator_noise_generator_);

  // Express body frame angular and linear velocities in world frame
  Eigen::Vector3d linear_velocity__inertialframe =
      orientation * linear_velocity__bodyframe;
  Eigen::Vector3d angular_velocity__inertialframe =
      orientation * angular_velocity__bodyframe;

  // Integrate translation
  position.x() += linear_velocity__inertialframe.x() * Dt;
  position.y() += linear_velocity__inertialframe.y() * Dt;
  position.z() += linear_velocity__inertialframe.z() * Dt;

  // Integrate angular velocity
  // (angular velocity --integrate--> angle axis --> quaternion)
  Eigen::Quaterniond orientation_delta;
  Eigen::Vector3d half_angle_vector =
      angular_velocity__inertialframe * Dt * 0.5;  // Half angle vector
  double half_angle = half_angle_vector.norm();    // Half angle
  if (half_angle > 0.001) {
    // This accurate update is only applied
    // if the angular velocity is significant (unstable otherwise)
    double term = std::sin(half_angle) / half_angle;
    // --> Term s.t. half_angle_vector * term = axis * sin(angle/2)
    orientation_delta.w() = std::cos(half_angle);  // cos(angle/2)
    orientation_delta.x() =
        half_angle_vector.x() * term;  // axis.x * sin(angle/2)
    orientation_delta.y() =
        half_angle_vector.y() * term;  // axis.y * sin(angle/2)
    orientation_delta.z() =
        half_angle_vector.z() * term;  // axis.z * sin(angle/2)
  } else {
    // This update is a first order approximation around zero
    // (avoids division by 0 when angle -> 0)
    orientation_delta.w() = 1;
    orientation_delta.x() = half_angle_vector.x();
    orientation_delta.y() = half_angle_vector.y();
    orientation_delta.z() = half_angle_vector.z();
  }
  orientation = orientation_delta * orientation;
  orientation.normalize();

  // Update stored pose
  odom_simulator_pose_stamped_.header = odometry_msg->header;
  tf::quaternionEigenToMsg(orientation,
                           odom_simulator_pose_stamped_.pose.orientation);
  tf::pointEigenToMsg(position, odom_simulator_pose_stamped_.pose.position);
  /* TODO(victorr): Broadcast the noisy odom as a TF
   *                instead of storing it as a variable,
   *                this way time history will be available
   *                -> important for pointcloudCallback() */
}
}  // namespace voxgraph