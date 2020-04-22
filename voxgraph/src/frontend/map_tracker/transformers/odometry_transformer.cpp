#include "voxgraph/frontend/map_tracker/transformers/odometry_transformer.h"

#include <string>

namespace voxgraph {
bool OdometryTransformer::waitForTransform(const ros::Time& frame_timestamp) {
  // Total time spent waiting for the updated pose
  ros::WallDuration t_waited(0.0);
  while (t_waited < transform_lookup_max_time_) {
    if (canTransform(frame_timestamp)) {
      return true;
    }
    transform_lookup_retry_period_.sleep();
    t_waited += transform_lookup_retry_period_;
    ros::spinOnce();
  }
  ROS_WARN(
      "Waited %.3fs, but still could not get an odometry transform for "
      "timestamp %.3fs",
      t_waited.toSec(), frame_timestamp.toSec());
  return false;
}

bool OdometryTransformer::lookupTransform(const ros::Time& frame_timestamp,
                                          Transformation* transform) {
  CHECK_NOTNULL(transform);
  if (!waitForTransform(frame_timestamp)) {
    return false;
  }
  auto iterator = odometry_queue_.upper_bound(frame_timestamp);

  // Ensure that the timestamp is not from before the buffer
  if (iterator == odometry_queue_.begin()) {
    ROS_WARN(
        "Requested transform from odom topic whose timstamp falls before "
        "any msg in the buffer.");
    return false;
  }

  // TODO(victorr): Implement interpolation using the exponential map
  // The interval's active submap id is stored at its start point
  iterator--;

  TransformationD transform_double;
  tf::poseMsgToKindr(iterator->second->pose.pose, &transform_double);
  *transform = transform_double.cast<float>();
  return true;
}

bool OdometryTransformer::lookupOdometryMsg(const ros::Time& frame_timestamp,
                                            nav_msgs::Odometry* odometry_msg) {
  CHECK_NOTNULL(odometry_msg);

  if (!waitForTransform(frame_timestamp)) {
    return false;
  }
  auto iterator = odometry_queue_.upper_bound(frame_timestamp);

  // Ensure that the timestamp is not from before the buffer
  if (iterator == odometry_queue_.begin()) {
    ROS_WARN(
        "Requested transform from odom topic whose timstamp falls before "
        "any msg in the buffer.");
    return false;
  }

  // TODO(victorr): Implement interpolation using the exponential map
  // The interval's active submap id is stored at its start point
  iterator--;

  *odometry_msg = *iterator->second;

  return true;
}

void OdometryTransformer::subscribeToTopic(ros::NodeHandle nh,
                                           const std::string& odometry_topic) {
  odometry_subscriber_ = nh.subscribe(
      odometry_topic, 1, &OdometryTransformer::odometryCallback, this);
}

void OdometryTransformer::odometryCallback(
    const nav_msgs::Odometry::ConstPtr& odometry_msg) {
  if (odometry_queue_.size() >= odometry_queue_length_ &&
      !odometry_queue_.empty()) {
    odometry_queue_.erase(odometry_queue_.begin());
  }
  odometry_queue_.emplace(odometry_msg->header.stamp, odometry_msg);
}
}  // namespace voxgraph
