//
// Created by victor on 10.04.19.
//

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosgraph_msgs/Clock.h>
#include <chrono>
#include <future>
#include <queue>
#include <string>
#include <thread>
#include <vector>
#include "voxgraph/frontend/voxgraph_mapper.h"
#include "voxgraph/tools/evaluation/map_evaluation.h"
#include "voxgraph/tools/odometry_simulator/odometry_simulator.h"

int main(int argc, char** argv) {
  using voxgraph::VoxgraphMapper;
  using voxgraph::OdometrySimulator;
  using voxgraph::VoxgraphSubmapCollection;
  using voxgraph::MapEvaluation;
  using std::future_status;
  using std::chrono::microseconds;

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
  std::queue<sensor_msgs::PointCloud2::ConstPtr> pointcloud_queue;
  std::future<int> mapper_async_handle;

  // Setup the odometry simulator
  OdometrySimulator odometry_simulator(nh, nh_private);
  topics_of_interest.emplace_back("/firefly/ground_truth/odometry");

  // Setup time related members
  bool playback_started(false);
  ros::Time playback_start_time(0);
  topics_of_interest.emplace_back("/clock");
  ros::Publisher clock_pub = nh.advertise<rosgraph_msgs::Clock>("/clock", 1);

  // Setup the map quality evaluation
  MapEvaluation map_evaluation(nh_private, ground_truth_tsdf_layer_path);

  // Process the bag
  for (rosbag::MessageInstance const read_msg :
       rosbag::View(bag, rosbag::TopicQuery(topics_of_interest))) {
    // Exit if CTRL+C was pressed
    if (!ros::ok()) {
      ROS_INFO("Shutting down...");
      bag.close();
      return -1;
    }

    // Process clock messages
    rosgraph_msgs::Clock::ConstPtr clock_msg =
        read_msg.instantiate<rosgraph_msgs::Clock>();
    if (clock_msg != nullptr) {
      if (playback_start_time == ros::Time(0)) {
        playback_start_time =
            clock_msg->clock + ros::Duration(skip_first_n_sec);
      } else {
        if (clock_msg->clock > playback_start_time) {
          playback_started = true;
        }
      }
      clock_pub.publish(clock_msg);
      continue;
    }

    // Start processing odometry and pointcloud msgs only after skip_first_n_sec
    if (playback_started) {
      // Process odometry messages
      nav_msgs::Odometry::ConstPtr odometry_msg =
          read_msg.instantiate<nav_msgs::Odometry>();
      if (odometry_msg != nullptr) {
        odometry_simulator.odometryCallback(odometry_msg);
        continue;
      }

      // Process pointcloud messages
      sensor_msgs::PointCloud2::ConstPtr pointcloud_msg =
          read_msg.instantiate<sensor_msgs::PointCloud2>();
      if (pointcloud_msg != nullptr) {
        // Add the pointcloud to the queue
        // NOTE: Even the first pointcloud is enqueued, since the mapper
        //       accesses them by const & (i.e. they must be kept in scope here)
        pointcloud_queue.emplace(pointcloud_msg);

        // If the mapper is already running, throttle it as required
        if (mapper_async_handle.valid()) {
          // If the mapper is busy but the queue is short, we continue
          auto status = mapper_async_handle.wait_for(microseconds(0));
          if (status != future_status::ready && pointcloud_queue.size() < 10) {
            continue;
          }
          // Otherwise we wait for the thread to finish
          mapper_async_handle.wait();
          // Once the thread is done, we can also cleanup its pointcloud
          pointcloud_queue.pop();
        }

        // Create a new thread operating on the queue's next pointcloud
        mapper_async_handle = std::async(std::launch::async, [&] {
          voxgraph_mapper.pointcloudCallback(pointcloud_queue.front());
          return 1;
        });
        continue;
      }
    }
  }

  // Evaluate the map by comparing it to ground truth
  ROS_INFO("Evaluating reconstruction error");
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
