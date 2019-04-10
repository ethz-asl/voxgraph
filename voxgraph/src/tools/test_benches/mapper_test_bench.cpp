//
// Created by victor on 10.04.19.
//

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <vector>
#include "voxgraph/frontend/voxgraph_mapper.h"
#include "voxgraph/tools/odometry_simulator/odometry_simulator.h"

int main(int argc, char** argv) {
  using voxgraph::VoxgraphMapper;
  using voxgraph::OdometrySimulator;

  // Start logging
  google::InitGoogleLogging(argv[0]);

  // Register with ROS master
  ros::init(argc, argv, "voxgraph_mapper_test_bench");

  // Create node handles
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Open the rosbag
  std::vector<std::string> topics_of_interest;
  rosbag::Bag bag;
  bag.open(
      "/home/victor/catkin_ws/bags/terrain_coverage_2019-01-21-23-26-31.bag");

  // Setup the mapper
  VoxgraphMapper voxgraph_mapper(nh, nh_private);
  topics_of_interest.emplace_back("/velodyne_points");

  // Setup the odometry simulator
  OdometrySimulator odometry_simulator(nh, nh_private);
  topics_of_interest.emplace_back("/firefly/ground_truth/odometry");

  // Setup the clock (used to skip the first n seconds)
  ros::Time start_time(0);
  bool playback_started(false);
  // TODO(victorr): Set this from ROS params
  ros::Duration skip_first_n_sec(50);
  topics_of_interest.emplace_back("/clock");

  // Process the bag
  for (rosbag::MessageInstance const m :
       rosbag::View(bag, rosbag::TopicQuery(topics_of_interest))) {
    // Exit if CTRL+C was pressed
    if (!ros::ok()) {
      ROS_INFO("Shutting down...");
      bag.close();
      return -1;
    }

    // Play the messages
    if (playback_started) {
      nav_msgs::Odometry::ConstPtr odometry_msg =
          m.instantiate<nav_msgs::Odometry>();
      if (odometry_msg != nullptr) {
        odometry_simulator.odometryCallback(odometry_msg);
        continue;
      }

      sensor_msgs::PointCloud2::ConstPtr pointcloud_msg =
          m.instantiate<sensor_msgs::PointCloud2>();
      if (pointcloud_msg != nullptr) {
        voxgraph_mapper.pointcloudCallback(pointcloud_msg);
        continue;
      }
    } else {
      // We're still waiting to reach the playback start time
      rosgraph_msgs::Clock::ConstPtr clock_msg =
          m.instantiate<rosgraph_msgs::Clock>();
      if (clock_msg != nullptr) {
        if (start_time == ros::Time(0)) {
          start_time = clock_msg->clock;
        } else {
          if (clock_msg->clock > start_time + skip_first_n_sec) {
            playback_started = true;
          }
        }
      }
    }
  }

  // Assess the map's quality
  // TODO(victorr): Compute difference between voxgraph map and GT map
  // TODO(victorr): Compute difference between voxgraph pose history and GT odom

  // Write to log
  // TODO(victorr): Write experiment results to log file

  // Close the rosbag
  bag.close();

  // Keep the ROS node alive in order to interact with its topics in Rviz
  std::cout << "Done" << std::endl;
  ros::spin();

  // Exit normally
  return 0;
}
