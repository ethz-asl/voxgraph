#include "voxgraph/frontend/map_tracker/map_tracker.h"
#include <limits>
#include <string>
#include <utility>
#include "voxgraph/tools/tf_helper.h"

namespace voxgraph {
MapTracker::MapTracker(VoxgraphSubmapCollection::ConstPtr submap_collection_ptr,
                       FrameNames frame_names, bool verbose)
    : scan_to_map_registerer_(std::move(submap_collection_ptr), true),
      frame_names_(std::move(frame_names)),
      tf_transformer_(),
      odom_transformer_(),
      verbose_(verbose) {}

void MapTracker::subscribeToTopics(ros::NodeHandle nh,
                                   const std::string &odometry_input_topic,
                                   const std::string &imu_biases_topic) {
  if (!odometry_input_topic.empty()) {
    ROS_INFO_STREAM("Using odometry from ROS topic: " << odometry_input_topic);
    use_odom_from_tfs_ = false;
    odom_transformer_.subscribeToTopic(nh, odometry_input_topic);
  }
  if (!imu_biases_topic.empty()) {
    imu_biases_subscriber_ =
        nh.subscribe(imu_biases_topic, 1, &MapTracker::imuBiasesCallback, this);
  }
}

void MapTracker::advertiseTopics(ros::NodeHandle nh_private,
                                 const std::string &odometry_output_topic) {
  odom_with_imu_biases_pub_ =
      nh_private.advertise<maplab_msgs::OdometryWithImuBiases>(
          odometry_output_topic, 3, false);
}

void MapTracker::imuBiasesCallback(
    const sensor_msgs::Imu::ConstPtr &imu_biases) {
  tf::vectorMsgToKindr(imu_biases->linear_acceleration, &forwarded_accel_bias_);
  tf::vectorMsgToKindr(imu_biases->angular_velocity, &forwarded_gyro_bias_);
}

bool MapTracker::updateToTime(const ros::Time &timestamp,
                              std::string sensor_frame_id) {
  // Keep track of the timestamp that the MapTracker is currently at
  current_timestamp_ = timestamp;

  // Update the odometry
  if (use_odom_from_tfs_) {
    // Update the odometry estimate
    if (!tf_transformer_.lookupTransform(frame_names_.odom_frame,
                                         frame_names_.base_link_frame,
                                         timestamp, &T_O_B_)) {
      return false;
    }
  } else {
    if (!odom_transformer_.lookupTransform(timestamp, &T_O_B_)) {
      return false;
    }
  }

  // Get the transformation from the pointcloud sensor to the robot's
  // base link from TFs, unless it was already provided through ROS params
  if (use_sensor_calibration_from_tfs_) {
    // TODO(victorr): Implement option to provide a sensor_frame_id instead of
    //                taking the one from the message
    // Strip leading slashes if needed to avoid TF errors
    if (sensor_frame_id[0] == '/') {
      sensor_frame_id = sensor_frame_id.substr(1, sensor_frame_id.length());
    }

    // Lookup the transform
    if (!tf_transformer_.lookupTransform(frame_names_.base_link_frame,
                                         sensor_frame_id, timestamp, &T_B_C_)) {
      return false;
    }
  }

  // Signal that all transforms were succesfully updated
  return true;
}

void MapTracker::updateWithLoopClosure(const Transformation &T_M_B_before,
                                       const Transformation &T_M_B_after) {
  T_M_L_ = T_M_B_after * T_M_B_before.inverse() * T_M_L_;
  ROS_INFO_STREAM("Applying pose correction:\n"
                  << T_M_B_after * T_M_B_before.inverse());
}

void MapTracker::registerPointcloud(
    const sensor_msgs::PointCloud2::Ptr &pointcloud_msg) {
  Transformation T_M_C_refined;
  bool pose_refinement_successful = scan_to_map_registerer_.refineSensorPose(
      pointcloud_msg, get_T_M_C(), &T_M_C_refined);
  if (pose_refinement_successful) {
    // Update the corrected odometry frame
    T_L_O_ =
        T_M_L_.inverse() * T_M_C_refined * T_B_C_.inverse() * T_O_B_.inverse();
  } else {
    ROS_WARN("Pose refinement failed");
  }
}

void MapTracker::publishTFs() {
  TfHelper::publishTransform(T_M_L_, frame_names_.mission_frame,
                             frame_names_.refined_frame_corrected, false,
                             current_timestamp_);
  TfHelper::publishTransform(T_L_O_, frame_names_.refined_frame_corrected,
                             frame_names_.odom_frame_corrected, false,
                             current_timestamp_);
  TfHelper::publishTransform(T_O_B_, frame_names_.odom_frame_corrected,
                             frame_names_.base_link_frame_corrected, false,
                             current_timestamp_);
  TfHelper::publishTransform(T_B_C_, frame_names_.base_link_frame_corrected,
                             frame_names_.sensor_frame_corrected, true,
                             current_timestamp_);
}

void MapTracker::publishOdometry() {
  if (odom_with_imu_biases_pub_.getNumSubscribers() > 0) {
    constexpr double double_min = std::numeric_limits<double>::lowest();
    if ((forwarded_accel_bias_.array() >= double_min).all() &&
        (forwarded_gyro_bias_.array() >= double_min).all()) {
      // Create the message and set its header
      maplab_msgs::OdometryWithImuBiases odometry_with_imu_biases;
      odometry_with_imu_biases.header.frame_id =
          frame_names_.refined_frame_corrected;
      odometry_with_imu_biases.header.stamp = current_timestamp_;
      odometry_with_imu_biases.child_frame_id = frame_names_.base_link_frame;
      odometry_with_imu_biases.odometry_state = 0u;

      // Set the odometry pose
      const Transformation refined_odometry = T_L_O_ * T_O_B_;
      tf::poseKindrToMsg(refined_odometry.cast<double>(),
                         &odometry_with_imu_biases.pose.pose);

      // Forward or hardcode the twist and covariances
      nav_msgs::Odometry closest_received_odom_msg_;
      if (!use_odom_from_tfs_
          && odom_transformer_.lookupOdometryMsg(current_timestamp_,
                                                 &closest_received_odom_msg_)) {
          // Forward the twist from ROVIO
          odometry_with_imu_biases.twist = closest_received_odom_msg_.twist;

          // Forward the pose and twist covariances
          odometry_with_imu_biases.pose.covariance =
              closest_received_odom_msg_.pose.covariance;
          odometry_with_imu_biases.twist.covariance =
              closest_received_odom_msg_.twist.covariance;
        } else {
          ROS_WARN("Could not find closest odometry msg. Will set twist to "
                   "zero and covariances of pose and twist to identity."
                   "Make sure to set a valid odometry topic in ROS parms.");

          // Set the twist to zero
          BiasVectorType zero_vector = BiasVectorType::Zero();
          tf::vectorKindrToMsg(zero_vector,
                                 &odometry_with_imu_biases.twist.twist.linear);
          tf::vectorKindrToMsg(zero_vector,
                                 &odometry_with_imu_biases.twist.twist.angular);

          // Set the pose and twist covariances to identity
          for (int row = 0; row < 6; ++row) {
            for (int col = 0; col < 6; ++col) {
              if (row == col) {
                odometry_with_imu_biases.twist.covariance[row * 6 + col] = 1.0;
                odometry_with_imu_biases.pose.covariance[row * 6 + col] = 1.0;
              } else {
                odometry_with_imu_biases.twist.covariance[row * 6 + col] = 0.0;
                odometry_with_imu_biases.pose.covariance[row * 6 + col] = 0.0;
              }
            }
          }
      }

      // Forward the biases from ROVIO
      tf::vectorKindrToMsg(forwarded_accel_bias_,
                           &odometry_with_imu_biases.accel_bias);
      tf::vectorKindrToMsg(forwarded_gyro_bias_,
                           &odometry_with_imu_biases.gyro_bias);

      // Publish the odom message
      odom_with_imu_biases_pub_.publish(odometry_with_imu_biases);
    }

    ROS_WARN("The voxgraph odometry output topic has subscribers, but no IMU "
             "biases have yet been received. Make sure to set a valid IMU "
             "biases topics in ROS params such that it can be forwarded");
  }
}
}  // namespace voxgraph
