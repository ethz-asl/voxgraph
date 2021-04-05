#ifndef VOXGRAPH_FRONTEND_MAP_TRACKER_TRANSFORMERS_ODOMETRY_TRANSFORMER_H_
#define VOXGRAPH_FRONTEND_MAP_TRACKER_TRANSFORMERS_ODOMETRY_TRANSFORMER_H_

#include <map>
#include <string>

#include <minkindr_conversions/kindr_msg.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include "voxgraph/common.h"

namespace voxgraph {
class OdometryTransformer {
 public:
  OdometryTransformer()
      : odometry_queue_length_(200),
        transform_lookup_retry_period_(0.02),
        transform_lookup_max_time_(0.25) {}

  inline bool canTransform(const ros::Time& timestamp) {
    return (!odometry_queue_.empty() &&
            timestamp < (--odometry_queue_.end())->second->header.stamp);
  }

  // Method that waits for a transform to become available
  bool waitForTransform(const ros::Time& frame_timestamp);

  // Method to lookup transforms and convert them Kindr
  bool lookupTransform(const ros::Time& frame_timestamp,
                       Transformation* transform);

  // Method to lookup the closest full odometry msg
  bool lookupOdometryMsg(const ros::Time& frame_timestamp,
                         nav_msgs::Odometry* odometry_msg);

  void subscribeToTopic(ros::NodeHandle nh, const std::string& odometry_topic);
  void odometryCallback(const nav_msgs::Odometry::ConstPtr& odometry_msg);

 private:
  ros::Subscriber odometry_subscriber_;

  size_t odometry_queue_length_;
  std::map<ros::Time, nav_msgs::Odometry::ConstPtr> odometry_queue_;

  // Transform lookup timers
  // Timeout between each update attempt
  const ros::WallDuration transform_lookup_retry_period_;
  // Maximum time to wait before giving up
  const ros::WallDuration transform_lookup_max_time_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_FRONTEND_MAP_TRACKER_TRANSFORMERS_ODOMETRY_TRANSFORMER_H_
