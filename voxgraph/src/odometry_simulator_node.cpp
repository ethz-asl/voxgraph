#include <glog/logging.h>
#include <ros/ros.h>

#include "voxgraph/tools/odometry_simulator/odometry_simulator.h"

int main(int argc, char** argv) {
  // Start logging
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  // Register with ROS master
  ros::init(argc, argv, "voxgraph_odometry_simulator");

  // Create node handles
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Create the mapper
  voxgraph::OdometrySimulator odometry_simulator(nh, nh_private);

  // Spin
  ros::spin();

  // Exit normally
  return 0;
}
