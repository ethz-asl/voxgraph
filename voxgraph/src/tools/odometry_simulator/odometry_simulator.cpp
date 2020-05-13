#include "voxgraph/tools/odometry_simulator/odometry_simulator.h"

#include <eigen_conversions/eigen_msg.h>
#include <minkindr_conversions/kindr_msg.h>
#include <tf2_ros/transform_broadcaster.h>
#include <voxgraph/common.h>

namespace voxgraph {
OdometrySimulator::OdometrySimulator(const ros::NodeHandle& nh,
                                     const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      debug_(false),
      subscriber_queue_length_(100),
      subscribe_to_odom_topic_("odometry"),
      published_mission_frame_("mission"),
      published_simulated_base_frame_("base"),
      published_original_base_frame_("base_ground_truth") {
  // Read odometry simulator params from ROS
  nh_private_.param("debug", debug_, debug_);
  nh_private_.param("subscriber_queue_length_", subscriber_queue_length_,
                    subscriber_queue_length_);
  nh_private_.param("subscribe_to_odom_topic", subscribe_to_odom_topic_,
                    subscribe_to_odom_topic_);
  nh_private_.param("published_mission_frame", published_mission_frame_,
                    published_mission_frame_);
  nh_private_.param("published_simulated_base_frame",
                    published_simulated_base_frame_,
                    published_simulated_base_frame_);
  nh_private_.param("published_original_base_frame",
                    published_original_base_frame_,
                    published_original_base_frame_);
  nh_private_.param("published_original_mission_frame",
                    published_original_mission_frame_,
                    published_original_mission_frame_);
  std::string published_odom_topic = "odometry_drifted";
  nh_private_.param("published_odom_topic", published_odom_topic,
                    published_odom_topic);
  int publisher_queue_length = 1;
  nh_private_.param("publisher_queue_length", publisher_queue_length,
                    publisher_queue_length);

  ros::NodeHandle nh_velocity_noise_(nh_private_, "odometry_noise/velocity");
  nh_velocity_noise_.param<double>("x/mean", noise_.x_vel.mean(), 0);
  nh_velocity_noise_.param<double>("x/stddev", noise_.x_vel.stddev(), 0);
  nh_velocity_noise_.param<double>("y/mean", noise_.y_vel.mean(), 0);
  nh_velocity_noise_.param<double>("y/stddev", noise_.y_vel.stddev(), 0);
  nh_velocity_noise_.param<double>("z/mean", noise_.z_vel.mean(), 0);
  nh_velocity_noise_.param<double>("z/stddev", noise_.z_vel.stddev(), 0);
  nh_velocity_noise_.param<double>("yaw/mean", noise_.yaw_rate.mean(), 0);
  nh_velocity_noise_.param<double>("yaw/stddev", noise_.yaw_rate.stddev(), 0);

  ros::NodeHandle nh_position_noise_(nh_private_, "odometry_noise/position");
  nh_position_noise_.param<double>("x/mean", noise_.x.mean(), 0);
  nh_position_noise_.param<double>("x/stddev", noise_.x.stddev(), 0);
  nh_position_noise_.param<double>("y/mean", noise_.y.mean(), 0);
  nh_position_noise_.param<double>("y/stddev", noise_.y.stddev(), 0);
  nh_position_noise_.param<double>("z/mean", noise_.z.mean(), 0);
  nh_position_noise_.param<double>("z/stddev", noise_.z.stddev(), 0);
  nh_position_noise_.param<double>("yaw/mean", noise_.yaw.mean(), 0);
  nh_position_noise_.param<double>("yaw/stddev", noise_.yaw.stddev(), 0);
  nh_position_noise_.param<double>("pitch/mean", noise_.pitch.mean(), 0);
  nh_position_noise_.param<double>("pitch/stddev", noise_.pitch.stddev(), 0);
  nh_position_noise_.param<double>("roll/mean", noise_.roll.mean(), 0);
  nh_position_noise_.param<double>("roll/stddev", noise_.roll.stddev(), 0);

  // Subscribe to the odometry ROS topic
  odometry_subscriber_ =
      nh_.subscribe(subscribe_to_odom_topic_,
                    static_cast<unsigned int>(subscriber_queue_length_),
                    &OdometrySimulator::odometryCallback, this);

  // Advertise the drifted odometry ROS topic
  odometry_drifted_publisher_ =
      nh_.advertise<nav_msgs::Odometry>(
          published_odom_topic,
          static_cast<unsigned int>(publisher_queue_length));
}

void OdometrySimulator::odometryCallback(
    const nav_msgs::Odometry::ConstPtr& odometry_msg) {
  // Check if this is the first odometry message
  if (internal_pose_.header.stamp.isZero()) {
    // Initialize the internal pose
    internal_pose_.header = odometry_msg->header;
    internal_pose_.pose = odometry_msg->pose.pose;
    // Initialize the published pose
    ROS_INFO("Initialized drifting odometry simulator");
    publishSimulatedPoseTf();
    publishSimulatedPoseMsg();
    return;
  }
  // Calculate time delta since last message
  double Dt =
      (odometry_msg->header.stamp - internal_pose_.header.stamp).toSec();

  // Load quantities of interest
  // Body position in mission frame
  Vector3 W_translation_WB;
  tf::pointMsgToKindr(internal_pose_.pose.position, &W_translation_WB);
  // Body orientation in the mission frame
  Rotation Rotation_WB;
  tf::quaternionMsgToKindr(internal_pose_.pose.orientation, &Rotation_WB);
  // Linear velocity in body frame
  Vector3 B_linear_velocity;
  tf::vectorMsgToEigen(odometry_msg->twist.twist.linear, B_linear_velocity);
  // Angular velocity in body frame
  Vector3 B_angular_velocity;
  tf::vectorMsgToEigen(odometry_msg->twist.twist.angular, B_angular_velocity);

  // Update the position
  // NOTE: This simulation does not account for correlated noise
  // Simulate noise on measured linear velocities
  B_linear_velocity.x() += noise_.x_vel();
  B_linear_velocity.y() += noise_.y_vel();
  B_linear_velocity.z() += noise_.z_vel();
  // Transform linear velocity into mission frame
  Vector3 W_linear_velocity =
      Rotation_WB.getRotationMatrix() * B_linear_velocity;
  // Integrate translation
  W_translation_WB.x() += W_linear_velocity.x() * Dt;
  W_translation_WB.y() += W_linear_velocity.y() * Dt;
  W_translation_WB.z() += W_linear_velocity.z() * Dt;

  // Update the orientation
  // NOTE: It is assumed that a good gravity estimate is available
  //       to eliminate pitch and roll drift
  // Transform angular velocity into mission frame
  Vector3 W_angular_velocity =
      Rotation_WB.getRotationMatrix() * B_angular_velocity;
  // Simulate noise on yaw rate
  // NOTE: This is done in mission frame to avoid affecting pitch and roll
  W_angular_velocity.z() += noise_.yaw_rate();
  // Transform angular velocity back into body frame
  B_angular_velocity =
      Rotation_WB.inverse().getRotationMatrix() * W_angular_velocity;
  // Integrate angular velocity
  Vector3 rotation_vector = W_angular_velocity * Dt;
  Rotation Rotation_WB_Delta(rotation_vector);
  Rotation_WB = Rotation_WB_Delta * Rotation_WB;
  Rotation_WB.normalize();

  // Update stored pose
  internal_pose_.header = odometry_msg->header;
  tf::quaternionKindrToMsg(Rotation_WB, &internal_pose_.pose.orientation);
  tf::pointKindrToMsg(W_translation_WB, &internal_pose_.pose.position);
  tf::vectorKindrToMsg(B_linear_velocity, &internal_twist_.linear);
  tf::vectorKindrToMsg(B_angular_velocity, &internal_twist_.angular);

  // Publish simulated pose TF
  publishSimulatedPoseTf();

  // Publish simulated pose msg
  publishSimulatedPoseMsg();

  // Publish original mission TF
  publishOriginalMissionTf(odometry_msg);

  // Publish true pose TF if requested
  if (debug_) {
    publishOriginalPoseTf(odometry_msg);
  }
}

void OdometrySimulator::publishSimulatedPoseTf() {
  // Declare the TF broadcaster
  static tf2_ros::TransformBroadcaster transform_broadcaster;

  // Update the TF message header
  published_pose_.header = internal_pose_.header;
  published_pose_.header.frame_id = published_mission_frame_;
  published_pose_.child_frame_id = published_simulated_base_frame_;

  // Get the rotation from the mission to body frame
  Rotation Rotation_WB;
  tf::quaternionMsgToKindr(internal_pose_.pose.orientation, &Rotation_WB);

  // Update the TF translation and add position noise (simulated in body frame)
  Vector3 B_translation_noise(noise_.x(), noise_.y(), noise_.z());
  Vector3 W_translation_noise =
      Rotation_WB.getRotationMatrix() * B_translation_noise;
  published_pose_.transform.translation.x =
      internal_pose_.pose.position.x + W_translation_noise.x();
  published_pose_.transform.translation.y =
      internal_pose_.pose.position.y + W_translation_noise.y();
  published_pose_.transform.translation.z =
      internal_pose_.pose.position.z + W_translation_noise.z();

  // Update the TF rotation and add orientation noise
  // TODO(victorr): Check if the assumption below is valid (for very small RPY)
  Vector3 noise_rotation_vector(noise_.roll(), noise_.pitch(), noise_.yaw());
  Rotation noise_quaternion(noise_rotation_vector);
  tf::quaternionKindrToMsg(noise_quaternion * Rotation_WB,
                           &published_pose_.transform.rotation);

  // Publish
  transform_broadcaster.sendTransform(published_pose_);  // Simulated pose
}

void OdometrySimulator::publishSimulatedPoseMsg() {
  // Write transform to an odometry message
  nav_msgs::Odometry drifted_odometry_msg;
  // Copy header
  drifted_odometry_msg.header = published_pose_.header;
  drifted_odometry_msg.header.frame_id = published_mission_frame_;
  drifted_odometry_msg.child_frame_id = published_simulated_base_frame_;
  // Copy position
  drifted_odometry_msg.pose.pose.position.x =
      published_pose_.transform.translation.x;
  drifted_odometry_msg.pose.pose.position.y =
      published_pose_.transform.translation.y;
  drifted_odometry_msg.pose.pose.position.z =
      published_pose_.transform.translation.z;
  // Copy orientation
  drifted_odometry_msg.pose.pose.orientation.w =
      published_pose_.transform.rotation.w;
  drifted_odometry_msg.pose.pose.orientation.x =
      published_pose_.transform.rotation.x;
  drifted_odometry_msg.pose.pose.orientation.y =
      published_pose_.transform.rotation.y;
  drifted_odometry_msg.pose.pose.orientation.z =
      published_pose_.transform.rotation.z;
  // Copy linear and angular velocities
  drifted_odometry_msg.twist.twist.linear = internal_twist_.linear;
  drifted_odometry_msg.twist.twist.angular = internal_twist_.angular;

  // Publish
  odometry_drifted_publisher_.publish(drifted_odometry_msg);
}

void OdometrySimulator::publishOriginalPoseTf(
    const nav_msgs::Odometry::ConstPtr& odometry_msg) {
  // Declare the TF broadcaster
  static tf2_ros::TransformBroadcaster transform_broadcaster;

  // Create the transform msg
  geometry_msgs::TransformStamped true_pose;
  true_pose.header = odometry_msg->header;
  true_pose.header.frame_id = published_mission_frame_;
  true_pose.child_frame_id = published_original_base_frame_;

  // Copy the true pose from the odometry message to the TF msg
  true_pose.transform.translation.x = odometry_msg->pose.pose.position.x;
  true_pose.transform.translation.y = odometry_msg->pose.pose.position.y;
  true_pose.transform.translation.z = odometry_msg->pose.pose.position.z;
  true_pose.transform.rotation = odometry_msg->pose.pose.orientation;

  // Publish
  transform_broadcaster.sendTransform(true_pose);
}

void OdometrySimulator::publishOriginalMissionTf(
    const nav_msgs::Odometry::ConstPtr& odometry_msg) {
  // Declare the TF broadcaster
  static tf2_ros::TransformBroadcaster transform_broadcaster;

  // Create the transform msg
  geometry_msgs::TransformStamped transform_msg;
  transform_msg.header = odometry_msg->header;
  transform_msg.header.frame_id = published_mission_frame_;
  transform_msg.child_frame_id = published_original_mission_frame_;

  // Ground truth world frame to robot frame (from odometry message)
  kindr::minimal::QuatTransformationTemplate<double> T_W_R;
  tf::poseMsgToKindr(odometry_msg->pose.pose, &T_W_R);
  // Simulated mission frame to robot frame (from simulated transform)
  kindr::minimal::QuatTransformationTemplate<double> T_O_R;
  tf::transformMsgToKindr(published_pose_.transform, &T_O_R);
  // Robot frame to Ground truth world frame
  kindr::minimal::QuatTransformationTemplate<double> T_O_W =
      T_O_R * T_W_R.inverse();
  tf::transformKindrToMsg(T_O_W, &transform_msg.transform);

  // Publish
  transform_broadcaster.sendTransform(transform_msg);
}
}  // namespace voxgraph
