#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosgraph_msgs/Clock.h>
#include <algorithm>
#include <chrono>
#include <future>
#include <map>
#include <memory>
#include <queue>
#include <string>
#include <thread>
#include <vector>
#include "voxgraph/frontend/voxgraph_mapper.h"
#include "voxgraph/tools/evaluation/map_evaluation.h"
#include "voxgraph/tools/odometry_simulator/odometry_simulator.h"

template <typename T>
std::string vectorToString(std::vector<T> vector) {
  std::stringstream ss;
  for (size_t i = 0; i < vector.size(); i++) {
    if (i != 0) {
      ss << ", ";
    }
    ss << vector[i];
  }
  return ss.str();
}

// TODO(victorr): Clean this test bench up and split it into smaller functions
// TODO(victorr): Use MessageInstance::getTopic() instead of directly trying to
//                instanciate. This would also ensure that the right topic is
//                used, instead of only filtering by msg type.
int main(int argc, char **argv) {
  using cblox::SubmapID;
  using voxblox::Transformation;
  using voxgraph::MapEvaluation;
  using voxgraph::OdometrySimulator;
  using voxgraph::PoseGraph;
  using voxgraph::VoxgraphMapper;
  using voxgraph::VoxgraphSubmap;
  using voxgraph::VoxgraphSubmapCollection;
  using TransformationDouble =
      kindr::minimal::QuatTransformationTemplate<double>;
  using geometry_msgs::PoseStamped;
  using geometry_msgs::PoseWithCovarianceStamped;
  using std::future_status;
  using std::chrono::duration_cast;
  using std::chrono::milliseconds;
  enum class GroundTruthPoseMsgType { Odometry, PoseWithCovarianceStamped };

  // Start logging
  google::InitGoogleLogging(argv[0]);

  // Register with ROS master
  ros::init(argc, argv, "voxgraph_mapper_test_bench");

  // Create node handles
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Load ROS params
  ROS_INFO("Loading ROS params");
  // Rosbag file path
  std::string rosbag_path;
  CHECK(nh_private.getParam("rosbag_filepath", rosbag_path));
  ROS_INFO_STREAM("- rosbag file path: " << rosbag_path);
  // Number of seconds to skip at the start of the rosbag
  double skip_first_n_sec;
  nh_private.param<double>("skip_first_n_sec", skip_first_n_sec, 0);
  ROS_INFO_STREAM("- skip first n seconds: " << skip_first_n_sec);
  // Registration method(s) to test
  std::vector<std::string> registration_methods;
  nh_private.param("registration_methods", registration_methods,
                   {"implicit_to_implicit"});
  ROS_INFO_STREAM(
      "- registration methods: " << vectorToString(registration_methods));
  // Sampling ratio(s) to test (in the registration constraints)
  std::vector<double> sampling_ratios;
  nh_private.param("sampling_ratios", sampling_ratios, {-1});
  ROS_INFO_STREAM(
      "- number of sampling ratios: " << vectorToString(sampling_ratios));
  // TF frame names
  std::string mission_frame;
  nh_private.param<std::string>("mission_frame", mission_frame, "mission");
  ROS_INFO_STREAM("- mission frame: " << mission_frame);
  std::string uncorrected_base_frame;
  nh_private.param<std::string>("uncorrected_base_frame",
                                uncorrected_base_frame, "robot");
  ROS_INFO_STREAM("- uncorrected robot pose frame: " << uncorrected_base_frame);
  std::string corrected_base_frame;
  nh_private.param<std::string>("corrected_base_frame", corrected_base_frame,
                                "robot_corrected");
  ROS_INFO_STREAM("- corrected robot pose frame: " << corrected_base_frame);
  // Pointcloud topic
  std::string pointcloud_topic;
  nh_private.param<std::string>("pointcloud_topic", pointcloud_topic,
                                "/velodyne_points");
  ROS_INFO_STREAM("- pointcloud topic: " << pointcloud_topic);
  // Input topic for odometry simulator (usually simulation ground truth)
  std::string odometry_simulator_input_topic;
  nh_private.param<std::string>("odom_simulator_input_topic",
                                odometry_simulator_input_topic, "");
  ROS_INFO_STREAM(
      "- odometry simulator input topic: " << odometry_simulator_input_topic);
  // Ground truth topic to use for trajectory error evaluation
  std::string ground_truth_pose_topic;
  nh_private.param<std::string>("ground_truth_pose_topic",
                                ground_truth_pose_topic,
                                "/firefly/ground_truth/odometry");
  ROS_INFO_STREAM("- ground truth pose topic: " << ground_truth_pose_topic);
  // Msg type of the ground truth topic defined above
  GroundTruthPoseMsgType ground_truth_pose_msg_type;
  std::string ground_truth_pose_msg_type_str;
  nh_private.param<std::string>("ground_truth_pose_msg_type",
                                ground_truth_pose_msg_type_str, "Odometry");
  if (ground_truth_pose_msg_type_str == "Odometry") {
    ground_truth_pose_msg_type = GroundTruthPoseMsgType::Odometry;
    ROS_INFO_STREAM("- ground truth pose msg type: Odometry");
  } else if (ground_truth_pose_msg_type_str == "PoseWithCovarianceStamped") {
    ground_truth_pose_msg_type =
        GroundTruthPoseMsgType::PoseWithCovarianceStamped;
    ROS_INFO_STREAM("- ground truth pose msg type: PoseWithCovarianceStamped");
  } else {
    ROS_FATAL_STREAM(
        "- ground truth pose msg type must be \"Odometry\" "
        "(default) or \"PoseWithCovarianceStamped\", but received "
        "\""
        << ground_truth_pose_msg_type_str << "\"");
    return -1;
  }
  // Max number of ground truth msgs to enqueue before dropping the oldest ones
  // NOTE: A longer queue means that more attempts will be made to lookup the
  //       uncorrected and corrected robot poses corresponding to the ground
  //       truth msg's timestamp. But keep in mind that the TF lookup buffer
  //       length is limited. Waiting too long before giving up can therefore
  //       result in all subsequent lookups failing.
  int max_ground_truth_queue_size;
  nh_private.param<int>("max_ground_truth_queue_size",
                        max_ground_truth_queue_size, 20);
  ROS_INFO_STREAM(
      "- max ground truth queue size: " << max_ground_truth_queue_size);
  // Ground truth decimation factor
  // NOTE: In simulation, the ground truth comes in at really high frequencies.
  //       It is therefore convenient to not log all msgs.
  int ground_truth_log_every_Nth_msg;  // Set to 1 to disable
  nh_private.param<int>("ground_truth_log_every_Nth_msg",
                        ground_truth_log_every_Nth_msg, 1);
  ROS_INFO_STREAM(
      "- ground truth log every n-th msg: " << ground_truth_log_every_Nth_msg);

  // Ground truth reconstruction TSDF file path (e.g. from voxblox_ground_truth)
  // Leave blank to skip reconstruction evaluation
  std::string ground_truth_tsdf_layer_path;
  nh_private.param<std::string>("ground_truth_tsdf_layer_filepath",
                                ground_truth_tsdf_layer_path, "");
  ROS_INFO_STREAM("- ground truth TSDF layer filepath: "
                  << (ground_truth_tsdf_layer_path.empty()
                          ? "none"
                          : ground_truth_tsdf_layer_path));

  // Create a log folder based on the current timestamp
  time_t raw_time = std::time(nullptr);
  struct tm time_struct;
  localtime_r(&raw_time, &time_struct);
  char time_char_buffer[80];
  strftime(time_char_buffer, 80, "%Y-%m-%d_%H-%M-%S", &time_struct);
  boost::filesystem::path log_folder_path;
  log_folder_path = "/home/victor/data/voxgraph/mapper_test_bench_stats";
  log_folder_path /= time_char_buffer;
  boost::filesystem::create_directory(log_folder_path);

  // Create and open the main log file
  boost::filesystem::path log_file_path = log_folder_path;
  log_file_path /= "main_log.csv";
  ROS_INFO_STREAM("Log file will be saved as: " << log_file_path);
  std::ofstream main_log_file;
  main_log_file.open(log_file_path.string());

  // Write the log file header
  main_log_file << "git_branch, git_hash, hw_concurrency, rosbag_filepath\n"
                << GIT_BRANCH << "," << GIT_COMMIT_HASH << ","
                << std::thread::hardware_concurrency() << "," << rosbag_path
                << std::endl;
  main_log_file
      << "id, registration_method, sampling_ratio, total_milliseconds, "
         "rmse, max_error, min_error, "
         "num_evaluated_voxels, num_ignored_voxels, "
         "num_overlapping_voxels, num_non_overlapping_voxels, "
         "average_num_residuals, max_num_residuals, "
         "average_solver_iterations, max_solver_iterations, "
         "average_successful_steps, max_successful_steps, "
         "average_unsuccessful_steps, max_unsuccessful_steps, "
         "average_solver_time, max_solver_time"
      << std::endl;

  // Define the logging format used for Eigen matrices
  Eigen::IOFormat ioformat(Eigen::StreamPrecision, Eigen::DontAlignCols, "; ",
                           "; ", "", "", "", "");

  // Loop through all params
  unsigned int experiment_id = 0;
  for (const std::string &registration_method : registration_methods) {
    for (const double &sampling_ratio : sampling_ratios) {
      // Set params specific to this run
      nh_private.setParam("measurements/submap_registration/sampling_ratio",
                          sampling_ratio);
      nh_private.setParam(
          "measurements/submap_registration/"
          "registration_method",
          registration_method);

      // Open the rosbag
      rosbag::Bag bag;
      bag.open(rosbag_path);
      std::vector<std::string> topics_of_interest;

      // Setup the mapper
      VoxgraphMapper voxgraph_mapper(nh, nh_private);
      topics_of_interest.emplace_back(pointcloud_topic);
      std::queue<sensor_msgs::PointCloud2::Ptr> pointcloud_queue;
      std::future<int> mapper_async_handle;

      // Setup the odometry simulator
      OdometrySimulator odometry_simulator(nh, nh_private);
      topics_of_interest.emplace_back(odometry_simulator_input_topic);

      // Setup TF publishing
      topics_of_interest.emplace_back("/tf");
      ros::Publisher tf_pub = nh.advertise<tf::tfMessage>("/tf", 3);

      // Setup time related members
      bool playback_started = (skip_first_n_sec == 0.0);
      ros::Time playback_start_time(0);
      topics_of_interest.emplace_back("/clock");
      ros::Publisher clock_pub =
          nh.advertise<rosgraph_msgs::Clock>("/clock", 1);

      // Setup the map quality evaluation
      std::unique_ptr<MapEvaluation> map_evaluation;
      if (!ground_truth_tsdf_layer_path.empty()) {
        map_evaluation.reset(
            new MapEvaluation(nh_private, ground_truth_tsdf_layer_path));
      }

      // Setup logging for the uncorrected and corrected position errors
      voxblox::Transformer transformer(nh, nh_private);
      topics_of_interest.emplace_back(ground_truth_pose_topic);
      std::queue<std::shared_ptr<PoseStamped>> ground_truth_pose_queue;
      // Transform from ground truth to mission frame
      TransformationDouble T_mission_D;
      ros::Time ground_truth_start_time;
      unsigned int ground_truth_counter = 0;
      boost::filesystem::path live_position_file_path = log_folder_path;
      live_position_file_path /=
          "pose_error_log-" + std::to_string(experiment_id) + ".csv";
      std::ofstream live_position_log_file;
      live_position_log_file.open(live_position_file_path.string());
      live_position_log_file << "timestamp, ground_truth_position, "
                                "uncorrected_position, corrected_position"
                             << std::endl;
      boost::filesystem::path final_pose_history_file_path = log_folder_path;
      final_pose_history_file_path /=
          "final_pose_history_log-" + std::to_string(experiment_id) + ".csv";
      std::ofstream final_pose_history_log_file;
      final_pose_history_log_file.open(final_pose_history_file_path.string());
      final_pose_history_log_file << "timestamp, final_corrected_position"
                                  << std::endl;

      // Process the bag
      auto start_time = std::chrono::steady_clock::now();
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

        // Process TF messages
        tf::tfMessage::ConstPtr tf_msg = read_msg.instantiate<tf::tfMessage>();
        if (tf_msg != nullptr) {
          tf_pub.publish(tf_msg);
          continue;
        }

        // Start processing odometry and pointcloud msgs only after
        // skip_first_n_sec
        if (playback_started) {
          // Log the position errors
          // Convert the ground truth pose msg to a common representation
          std::shared_ptr<PoseStamped> ground_truth_pose_msg;
          switch (ground_truth_pose_msg_type) {
            case GroundTruthPoseMsgType::Odometry: {
              nav_msgs::Odometry::ConstPtr msg =
                  read_msg.instantiate<nav_msgs::Odometry>();
              if (msg) {
                ground_truth_pose_msg = std::make_shared<PoseStamped>();
                ground_truth_pose_msg->header = msg->header;
                ground_truth_pose_msg->pose = msg->pose.pose;
              }
              break;
            }
            case GroundTruthPoseMsgType::PoseWithCovarianceStamped: {
              PoseWithCovarianceStamped::ConstPtr msg =
                  read_msg.instantiate<PoseWithCovarianceStamped>();
              if (msg) {
                ground_truth_pose_msg = std::make_shared<PoseStamped>();
                ground_truth_pose_msg->header = msg->header;
                ground_truth_pose_msg->pose = msg->pose.pose;
              }
              break;
            }
          }
          // Process the ground truth msg
          if (ground_truth_pose_msg) {
            // Check if this is the first received ground truth msg
            if (ground_truth_pose_queue.empty()) {
              // Initialize transform from ground truth to mission frame
              TransformationDouble T_D_mission;
              tf::poseMsgToKindr(ground_truth_pose_msg->pose, &T_D_mission);
              T_mission_D = T_D_mission.inverse();
              // Set its roll and pitch to zero
              TransformationDouble::Vector3 Q_vec =
                  T_mission_D.getRotation().log();
              Q_vec[0] = Q_vec[1] = 0;
              T_mission_D.getRotation() =
                  TransformationDouble::Rotation::exp(Q_vec);
              // Get the start time
              ground_truth_start_time = ground_truth_pose_msg->header.stamp;
            }

            // Only process every n-th ground truth message
            if (ground_truth_counter % ground_truth_log_every_Nth_msg == 0) {
              // Add the received ground truth pose to the queue
              ground_truth_pose_queue.emplace(ground_truth_pose_msg);

              // Get the time and pose of the oldest msg in the queue
              const ros::Time query_time =
                  ground_truth_pose_queue.front()->header.stamp;
              TransformationDouble T_D_ground_truth_pose;
              tf::poseMsgToKindr(ground_truth_pose_queue.front()->pose,
                                 &T_D_ground_truth_pose);
              const TransformationDouble T_mission_gt_pose =
                  T_mission_D * T_D_ground_truth_pose;

              // Try to lookup the corresponding corrected and uncorrected pose
              Transformation T_mission_base_corrected;
              Transformation T_mission_base_uncorrected;
              bool corrected_pose_found = transformer.lookupTransform(
                  corrected_base_frame, mission_frame, query_time,
                  &T_mission_base_corrected);
              bool uncorrected_pose_found = transformer.lookupTransform(
                  uncorrected_base_frame, mission_frame, query_time,
                  &T_mission_base_uncorrected);

              if (corrected_pose_found && uncorrected_pose_found) {
                // Pop the ground truth pose msg, now that we found its
                // corresponding corrected and uncorrected robot poses
                ground_truth_pose_queue.pop();

                // Log poses to file
                live_position_log_file
                    << (query_time - ground_truth_start_time).toSec() << ","
                    << T_mission_gt_pose.getPosition().format(ioformat) << ","
                    << T_mission_base_uncorrected.getPosition().format(ioformat)
                    << ","
                    << T_mission_base_corrected.getPosition().format(ioformat)
                    << std::endl;
              } else if (ground_truth_pose_queue.size() >
                         max_ground_truth_queue_size) {
                // If the queue becomes to large we give up on the oldest ground
                // truth msg, even if we haven't found its corresponding robot
                // poses. This is necessary since we might otherwise exceed the
                // TF buffer length for the remaining poses (i.e. lose them all)
                ROS_WARN_STREAM("Could not find transforms from "
                                << uncorrected_base_frame << " and/or "
                                << corrected_base_frame << " to "
                                << mission_frame << " at time " << query_time);
                ground_truth_pose_queue.pop();
              }
            }

            ground_truth_counter++;
            // NOTE: We don't call 'continue', since the odometry simulator
            //       input could come from the same ground truth topic
          }

          // Process odometry messages
          nav_msgs::Odometry::ConstPtr odometry_msg =
              read_msg.instantiate<nav_msgs::Odometry>();
          if (odometry_msg != nullptr) {
            odometry_simulator.odometryCallback(odometry_msg);
            continue;
          }

          // Process pointcloud messages
          sensor_msgs::PointCloud2::Ptr pointcloud_msg =
              read_msg.instantiate<sensor_msgs::PointCloud2>();
          if (pointcloud_msg != nullptr) {
            // Add the pointcloud to the queue
            // NOTE: Even the first pointcloud is enqueued, since the mapper
            //       accesses them by const ref (i.e. they must be kept in
            //       scope here)
            pointcloud_queue.emplace(pointcloud_msg);

            // If the mapper is already running, throttle it as required
            if (mapper_async_handle.valid()) {
              // If the mapper is busy but the queue is short, we continue
              auto status = mapper_async_handle.wait_for(milliseconds(0));
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
              // Process the pointcloud
              voxgraph_mapper.pointcloudCallback(pointcloud_queue.front());
              return 1;
            });

            continue;
          }
        }
      }
      auto end_time = std::chrono::steady_clock::now();

      // Get a const ref to the submap collection
      const VoxgraphSubmapCollection &submap_collection =
          voxgraph_mapper.getSubmapCollection();

      // Evaluate the reconstruction error
      MapEvaluation::EvaluationDetails evaluation_result;
      if (!ground_truth_tsdf_layer_path.empty()) {
        // Evaluate reconstruction error by comparing the map to ground truth
        CHECK_NOTNULL(map_evaluation);
        ROS_INFO("Evaluating reconstruction error");
        evaluation_result = map_evaluation->evaluate(submap_collection);
      }

      // Summarize the pose graph optimization cost
      int average_num_residuals = 0;
      int max_num_residuals = 0;
      int average_solver_iterations = 0;
      size_t max_solver_iterations = 0;
      int average_successful_steps = 0;
      int max_successful_steps = 0;
      int average_unsuccessful_steps = 0;
      int max_unsuccessful_steps = 0;
      double average_solver_time = 0;
      double max_solver_time = 0;
      {
        PoseGraph::SolverSummaryList solver_summaries =
            voxgraph_mapper.getSolverSummaries();
        for (const ceres::Solver::Summary &solver_summary : solver_summaries) {
          // Update the averages
          average_num_residuals += solver_summary.num_residuals;
          average_solver_iterations += solver_summary.iterations.size();
          average_successful_steps += solver_summary.num_successful_steps;
          average_unsuccessful_steps += solver_summary.num_unsuccessful_steps;
          average_solver_time += solver_summary.total_time_in_seconds;
          // Update the maxima
          max_num_residuals =
              std::max(solver_summary.num_residuals, max_num_residuals);
          max_solver_iterations =
              std::max(solver_summary.iterations.size(), max_solver_iterations);
          max_successful_steps = std::max(solver_summary.num_successful_steps,
                                          max_successful_steps);
          max_unsuccessful_steps = std::max(
              solver_summary.num_unsuccessful_steps, max_unsuccessful_steps);
          max_solver_time =
              std::max(solver_summary.total_time_in_seconds, max_solver_time);
        }
        // Finish the averages
        const int &num_summaries = solver_summaries.size();
        average_num_residuals /= num_summaries;
        average_solver_iterations /= num_summaries;
        average_successful_steps /= num_summaries;
        average_unsuccessful_steps /= num_summaries;
        average_solver_time /= num_summaries;
      }

      // Write to log
      main_log_file
          << experiment_id << "," << registration_method << ","
          << sampling_ratio << ","
          << duration_cast<milliseconds>(end_time - start_time).count() << ","
          << evaluation_result.reconstruction.rmse << ","
          << evaluation_result.reconstruction.max_error << ","
          << evaluation_result.reconstruction.min_error << ","
          << evaluation_result.reconstruction.num_evaluated_voxels << ","
          << evaluation_result.reconstruction.num_ignored_voxels << ","
          << evaluation_result.reconstruction.num_overlapping_voxels << ","
          << evaluation_result.reconstruction.num_non_overlapping_voxels << ","
          << average_num_residuals << "," << max_num_residuals << ","
          << average_solver_iterations << "," << max_solver_iterations << ","
          << average_successful_steps << "," << max_successful_steps << ","
          << average_unsuccessful_steps << "," << max_unsuccessful_steps << ","
          << average_solver_time << "," << max_solver_time << std::endl;

      // Log the final corrected pose history to a file
      VoxgraphSubmapCollection::PoseStampedVector final_pose_history =
          voxgraph_mapper.getSubmapCollection().getPoseHistory();
      for (const auto &stamped_pose : final_pose_history) {
        TransformationDouble pose;
        tf::poseMsgToKindr(stamped_pose.pose, &pose);
        final_pose_history_log_file
            << (stamped_pose.header.stamp - ground_truth_start_time).toSec()
            << "," << pose.getPosition().format(ioformat) << std::endl;
      }

      // Close the rosbag
      bag.close();
      live_position_log_file.close();
      final_pose_history_log_file.close();
      experiment_id++;
    }
  }

  // Close the log file
  main_log_file.close();

  // Keep the ROS node alive in order to interact with its topics in Rviz
  std::cout << "Done" << std::endl;
  ros::spin();

  // Exit normally
  return 0;
}
