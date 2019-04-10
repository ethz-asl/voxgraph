//
// Created by victor on 10.04.19.
//

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosgraph_msgs/Clock.h>
#include <string>
#include <vector>
#include "voxgraph/frontend/voxgraph_mapper.h"
#include "voxgraph/tools/evaluation/map_evaluation.h"
#include "voxgraph/tools/odometry_simulator/odometry_simulator.h"

int main(int argc, char** argv) {
  using voxgraph::VoxgraphMapper;
  using voxgraph::OdometrySimulator;
  using voxgraph::VoxgraphSubmapCollection;
  using voxgraph::MapEvaluation;

  // Start logging
  google::InitGoogleLogging(argv[0]);

  // Register with ROS master
  ros::init(argc, argv, "voxgraph_mapper_test_bench");

  // Create node handles
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Load ROS params
  std::string rosbag_path, ground_truth_tsdf_layer_path;
  CHECK(nh_private.getParam("rosbag_filepath", rosbag_path));
  CHECK(nh_private.getParam("map_evaluation/ground_truth_tsdf_layer_filepath",
                            ground_truth_tsdf_layer_path));
  double skip_first_n_sec;
  nh_private.param<double>("skip_first_n_sec", skip_first_n_sec, 0);

  // Open the rosbag
  rosbag::Bag bag;
  bag.open(rosbag_path);
  std::vector<std::string> topics_of_interest;

  // Setup the mapper
  VoxgraphMapper voxgraph_mapper(nh, nh_private);
  topics_of_interest.emplace_back("/velodyne_points");

  // Setup the odometry simulator
  OdometrySimulator odometry_simulator(nh, nh_private);
  topics_of_interest.emplace_back("/firefly/ground_truth/odometry");

  // Setup the clock (used to skip the first n seconds)
  bool playback_started(false);
  ros::Time playback_start_time(0);
  topics_of_interest.emplace_back("/clock");

  // Setup the map quality evaluation
  MapEvaluation map_evaluation(nh_private, ground_truth_tsdf_layer_path);

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
        if (playback_start_time == ros::Time(0)) {
          playback_start_time =
              clock_msg->clock + ros::Duration(skip_first_n_sec);
        } else {
          if (clock_msg->clock > playback_start_time) {
            playback_started = true;
          }
        }
      }
    }
  }

  // Evaluate the map by comparing it to ground truth
  map_evaluation.evaluate(voxgraph_mapper.getSubmapCollection());

  // Evalute the drift correction by comparing the pose history to ground truth
  // TODO(victorr): Implement this

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
