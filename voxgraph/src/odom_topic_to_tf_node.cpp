#include <glog/logging.h>
#include <maplab_msgs/OdometryWithImuBiases.h>
#include <minkindr_conversions/kindr_msg.h>
#include <ros/ros.h>
#include <voxgraph/common.h>
#include "voxgraph/tools/tf_helper.h"

void odometryCallback(
    const maplab_msgs::OdometryWithImuBiases::ConstPtr& odometry_msg) {
  voxgraph::TransformationD T_odom_base_link;
  tf::poseMsgToKindr(odometry_msg->pose.pose, &T_odom_base_link);
  voxgraph::TfHelper::publishTransform(T_odom_base_link.cast<float>(), "odom",
                                       "imu", false,
                                       odometry_msg->header.stamp);
  voxgraph::TfHelper::publishTransform(voxblox::Transformation(), "mission",
                                       "odom", false,
                                       odometry_msg->header.stamp);
}

int main(int argc, char** argv) {
  // Start logging
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  // Register with ROS master
  ros::init(argc, argv, "odom_topic_to_tf_node");

  // Create node handles
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Create the odometry msg to TF republisher
  ros::Subscriber odom_sub =
      nh.subscribe("/msf_core/maplab_odometry", 1, &odometryCallback);

  // Spin
  ros::spin();

  // Exit normally
  return 0;
}
