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
  std::vector<std::string> registration_methods;
  nh_private.param("registration_methods", registration_methods,
                   {"implicit_to_implicit"});
  std::vector<double> sampling_ratios;
  nh_private.param("sampling_ratios", sampling_ratios, {-1});

  // Create the log file with the current timestamp as its file name
  time_t raw_time = std::time(nullptr);
  struct tm time_struct;
  localtime_r(&raw_time, &time_struct);
  char time_char_buffer[80];
  strftime(time_char_buffer, 80, "%Y-%m-%d_%H-%M-%S", &time_struct);
  boost::filesystem::path log_file_path;
  log_file_path = "/home/victor/data/voxgraph/mapper_test_bench_stats";
  log_file_path /= time_char_buffer;
  log_file_path += ".csv";
  ROS_INFO_STREAM("Log file will be saved as: " << log_file_path);
  std::ofstream log_file;
  log_file.open(log_file_path.string());

  // Write the log file header
  log_file << "git_branch, git_hash, hw_concurrency\n"
           << GIT_BRANCH << "," << GIT_COMMIT_HASH << ","
           << std::thread::hardware_concurrency() << std::endl;
  log_file << "registration_method, sampling_ratio, total_milliseconds, "
              "rmse, max_error, min_error, "
              "num_evaluated_voxels, num_ignored_voxels, "
              "num_overlapping_voxels, num_non_overlapping_voxels"
           << std::endl;

  // Loop through all params
  for (double sampling_ratio : sampling_ratios) {
    // Set params specific to this run
    nh_private.setParam("measurements/submap_registration/sampling_ratio",
                        sampling_ratio);
    auto start_time = std::chrono::steady_clock::now();

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

      // Start processing odometry and pointcloud msgs only after
      // skip_first_n_sec
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
          //       accesses them by const & (i.e. they must be kept in scope
          //       here)
          pointcloud_queue.emplace(pointcloud_msg);

          // If the mapper is already running, throttle it as required
          if (mapper_async_handle.valid()) {
            // If the mapper is busy but the queue is short, we continue
            auto status = mapper_async_handle.wait_for(microseconds(0));
            if (status != future_status::ready &&
                pointcloud_queue.size() < 10) {
              continue;
            }
            // Otherwise we wait for the thread to finish
            mapper_async_handle.wait();
            // Once the thread is done, we can also cleanup its pointcloud
            pointcloud_queue.pop();
          }

          // Launch processing of the queue's next ptcloud as an asynchronous
          // task
          mapper_async_handle = std::async(std::launch::async, [&] {
            voxgraph_mapper.pointcloudCallback(pointcloud_queue.front());
            return 1;
          });
          continue;
        }
      }
    }

    auto end_time = std::chrono::steady_clock::now();

    // Evaluate the map by comparing it to ground truth
    ROS_INFO("Evaluating reconstruction error");
    auto evaluation_details =
        map_evaluation.evaluate(voxgraph_mapper.getSubmapCollection());

    // Evalute the drift correction by comparing the pose history to ground
    // truth
    // TODO(victorr): Implement this

    // Write to log
    std::string registration_method;
    nh_private.getParam("measurements/submap_registration/registration_method",
                        registration_method);
    log_file << registration_method << "," << sampling_ratio << ","
             << std::chrono::duration_cast<std::chrono::milliseconds>(
                    end_time - start_time)
                    .count()
             << "," << evaluation_details.rmse << ","
             << evaluation_details.max_error << ","
             << evaluation_details.min_error << ","
             << evaluation_details.num_evaluated_voxels << ","
             << evaluation_details.num_ignored_voxels << ","
             << evaluation_details.num_overlapping_voxels << ","
             << evaluation_details.num_non_overlapping_voxels << std::endl;

    // Close the rosbag
    bag.close();
  }

  // Close the log file
  log_file.close();

  // Keep the ROS node alive in order to interact with its topics in Rviz
  std::cout << "Done" << std::endl;
  ros::spin();

  // Exit normally
  return 0;
}
