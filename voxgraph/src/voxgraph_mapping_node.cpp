//
// Created by victor on 16.11.18.
//

#include <glog/logging.h>
#include <ros/ros.h>
#include "voxgraph/frontend/voxgraph_mapper.h"

int main(int argc, char** argv) {
  // Start logging
  google::InitGoogleLogging(argv[0]);

  // Register with ROS master
  ros::init(argc, argv, "voxgraph_mapping");

  // Create node handles
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Create the mapper
  voxgraph::VoxgraphMapper voxgraph_mapper(nh, nh_private);

  // Spin
  ros::spin();

  // Exit normally
  return 0;
}
