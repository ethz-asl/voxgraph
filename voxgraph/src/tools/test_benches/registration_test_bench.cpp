#include <time.h>

#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <cblox/core/submap_collection.h>
#include <cblox/io/submap_io.h>
#include <cblox/utils/quat_transformation_protobuf_utils.h>
#include <glog/logging.h>
#include <ros/ros.h>
#include <voxblox/io/layer_io.h>
#include <voxblox_ros/ptcloud_vis.h>
#include <voxblox_ros/ros_params.h>

#include "voxgraph/frontend/submap_collection/voxgraph_submap.h"
#include "voxgraph/tools/submap_registration_helper.h"
#include "voxgraph/tools/tf_helper.h"
#include "voxgraph/tools/visualization/submap_visuals.h"

int main(int argc, char** argv) {
  enum SolverReportStyle { kBrief, kFull, kNone };
  using voxblox::Transformation;
  using voxgraph::RegistrationCostFunction;
  using voxgraph::SubmapRegistrationHelper;
  using voxgraph::TfHelper;
  using voxgraph::VoxgraphSubmap;

  // Start logging
  google::InitGoogleLogging(argv[0]);

  // Register with ROS master
  ros::init(argc, argv, "voxgraph_registration_test_bench");

  // Create node handles
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Read ROS params: General experiment settings
  std::string submap_collection_file_path, log_folder_path;
  CHECK(nh_private.getParam("submap_collection_file_path",
                            submap_collection_file_path))
      << "Rosparam submap_collection_file_path must be set" << std::endl;
  CHECK(nh_private.getParam("log_folder_path", log_folder_path))
      << "Rosparam log_folder_path must be set" << std::endl;
  cblox::SubmapID reference_submap_id, reading_submap_id;
  {
    int fixed_submap_id_tmp, reading_submap_id_tmp;
    CHECK(nh_private.getParam("reference_submap_id", fixed_submap_id_tmp))
        << "Rosparam reference_submap_id must be set" << std::endl;
    CHECK(nh_private.getParam("reading_submap_id", reading_submap_id_tmp))
        << "Rosparam reading_submap_id must be set" << std::endl;
    reference_submap_id = static_cast<cblox::SubmapID>(fixed_submap_id_tmp);
    reading_submap_id = static_cast<cblox::SubmapID>(reading_submap_id_tmp);
  }
  std::vector<float> range_x, range_y, range_z;
  std::vector<float> range_yaw, range_pitch, range_roll;
  nh_private.param("test_range/x", range_x, {0});
  nh_private.param("test_range/y", range_y, {0});
  nh_private.param("test_range/z", range_z, {0});
  nh_private.param("test_range/yaw", range_yaw, {0});
  nh_private.param("test_range/pitch", range_pitch, {0});
  nh_private.param("test_range/roll", range_roll, {0});
  SolverReportStyle solver_report_style;
  {
    std::string report_style_str;
    nh_private.param<std::string>("solver_report_style", report_style_str,
                                  "brief");
    if (report_style_str == "brief") {
      solver_report_style = kBrief;
    } else if (report_style_str == "full") {
      solver_report_style = kFull;
    } else if (report_style_str == "none") {
      solver_report_style = kNone;
    } else {
      ROS_FATAL(
          "Param \"report_style\" must be "
          "\"brief\" (default), \"full\" or \"none\"");
      ros::shutdown();
      return -1;
    }
  }

  // Read ROS params: Submap registration settings
  ros::NodeHandle nh_registration(nh_private, "submap_registration");
  VoxgraphSubmap::Config::RegistrationFilter registration_filter_config;
  SubmapRegistrationHelper::Options registerer_options;
  nh_registration.param("solver/max_num_iterations",
                        registerer_options.solver.max_num_iterations, 40);
  nh_registration.param("solver/parameter_tolerance",
                        registerer_options.solver.parameter_tolerance, 3e-9);
  nh_registration.param("cost/no_correspondence_cost",
                        registerer_options.registration.no_correspondence_cost,
                        registerer_options.registration.no_correspondence_cost);
  nh_registration.param("cost/use_esdf_distance",
                        registerer_options.registration.use_esdf_distance,
                        registerer_options.registration.use_esdf_distance);
  nh_registration.param("cost/visualize_residuals",
                        registerer_options.registration.visualize_residuals,
                        registerer_options.registration.visualize_residuals);
  nh_registration.param("cost/visualize_gradients",
                        registerer_options.registration.visualize_gradients,
                        registerer_options.registration.visualize_gradients);
  nh_registration.param("cost/visualize_transforms",
                        registerer_options.registration.visualize_transforms_,
                        registerer_options.registration.visualize_transforms_);
  nh_registration.param("cost/sampling_ratio",
                        registerer_options.registration.sampling_ratio,
                        registerer_options.registration.sampling_ratio);
  // Get the registration method (from string)
  {
    std::string registration_method_str;
    nh_registration.param<std::string>("cost/registration_method",
                                       registration_method_str,
                                       "implicit_to_implicit");
    if (registration_method_str == "implicit_to_implicit") {
      registerer_options.registration.registration_point_type =
          VoxgraphSubmap::RegistrationPointType::kVoxels;
    } else if (registration_method_str == "explicit_to_implicit") {
      registerer_options.registration.registration_point_type =
          VoxgraphSubmap::RegistrationPointType::kIsosurfacePoints;
    } else {
      ROS_FATAL(
          "Param \"submap_registration/cost/registration_method\" must be "
          "\"implicit_to_implicit\" (default) or \"explicit_to_implicit\"");
      ros::shutdown();
      return -1;
    }
  }
  // Get the Jacobian evaluation method (from string)
  {
    std::string jacobian_evaluation_method_str;
    nh_registration.param<std::string>("cost/jacobian_evaluation_method",
                                       jacobian_evaluation_method_str,
                                       "analytic");
    if (jacobian_evaluation_method_str == "analytic") {
      registerer_options.registration.jacobian_evaluation_method =
          RegistrationCostFunction::JacobianEvaluationMethod::kAnalytic;
    } else if (jacobian_evaluation_method_str == "numeric") {
      registerer_options.registration.jacobian_evaluation_method =
          RegistrationCostFunction::JacobianEvaluationMethod::kNumeric;
    } else {
      ROS_FATAL(
          "Param \"submap_registration/cost/jacobian_evaluation_method\" "
          "must be \"analytic\" (default) or \"numeric\"");
      ros::shutdown();
      return -1;
    }
  }
  // Get the submap relevant voxel thresholds
  nh_registration.param("cost/min_voxel_weight",
                        registration_filter_config.min_voxel_weight,
                        registration_filter_config.min_voxel_weight);
  nh_registration.param("cost/max_voxel_distance",
                        registration_filter_config.max_voxel_distance,
                        registration_filter_config.max_voxel_distance);

  // Announce ROS topics for Rviz debug visuals
  ros::Publisher reference_mesh_pub =
      nh_private.advertise<visualization_msgs::Marker>("reference_mesh_pub", 1,
                                                       true);
  ros::Publisher reference_tsdf_pub =
      nh_private.advertise<pcl::PointCloud<pcl::PointXYZI>>(
          "reference_tsdf_pointcloud", 1, true);
  ros::Publisher perturbed_reading_mesh_pub =
      nh_private.advertise<visualization_msgs::Marker>(
          "perturbed_reading_mesh_pub", 1, true);
  ros::Publisher optimized_reading_mesh_pub =
      nh_private.advertise<visualization_msgs::Marker>(
          "optimized_reading_mesh_pub", 1, true);

  // Load the submap collection
  cblox::SubmapCollection<VoxgraphSubmap>::Ptr submap_collection_ptr;
  cblox::io::LoadSubmapCollection<VoxgraphSubmap>(submap_collection_file_path,
                                                  &submap_collection_ptr);

  // If both submaps IDs are the same, duplicate the reference submap
  if (reference_submap_id == reading_submap_id) {
    ROS_INFO(
        "Reference and reading submap IDs are the same, "
        "duplicating the reference...");
    reading_submap_id = INT32_MAX;
    CHECK(submap_collection_ptr->duplicateSubmap(reference_submap_id,
                                                 reading_submap_id));
  }

  // Setup the submap to submap registerer
  SubmapRegistrationHelper submap_registerer(submap_collection_ptr,
                                             registerer_options);

  // Setup visualization tools
  voxgraph::SubmapVisuals submap_vis(
      submap_collection_ptr->getConfig(),
      voxblox::getMeshIntegratorConfigFromRosParam(nh_private));

  // Save the reference submap pose and the original reading submap pose
  Transformation transform_getter;
  CHECK(submap_collection_ptr->getSubmapPose(reference_submap_id,
                                             &transform_getter));
  const Transformation T_mission__reference = transform_getter;
  CHECK(submap_collection_ptr->getSubmapPose(reading_submap_id,
                                             &transform_getter));
  const Transformation T_mission__reading_original = transform_getter;
  const Eigen::Vector3f& ground_truth_position =
      T_mission__reading_original.getPosition();

  // Publish TFs for the reference and reading submap
  TfHelper::publishTransform(T_mission__reference, "mission",
                             "reference_submap", true);
  TfHelper::publishTransform(T_mission__reading_original, "mission",
                             "reading_submap", true);

  // Publish the meshes used to visualize the submaps
  {
    // Wait for Rviz to launch so that it receives the meshes
    ros::Rate wait_rate(1);
    while (reference_mesh_pub.getNumSubscribers() == 0) {
      ROS_INFO(
          "Waiting for Rviz to launch "
          "and subscribe to topic 'reference_mesh_pub'");
      wait_rate.sleep();
    }
    // Publish reference submap Mesh
    submap_vis.publishMesh(*submap_collection_ptr, reference_submap_id,
                           voxblox::Color::Green(), "reference_submap",
                           reference_mesh_pub);
    // Publish temporary TFs for the moving meshes, such that Rviz
    // doesn't discard them due to missing frame position information
    TfHelper::publishTransform(T_mission__reading_original, "mission",
                               "perturbed_submap", true);
    TfHelper::publishTransform(T_mission__reading_original, "mission",
                               "optimized_submap", true);
    // Publish the reading submap mesh used to indicate its perturbed pose
    submap_vis.publishMesh(*submap_collection_ptr, reading_submap_id,
                           voxblox::Color::Red(), "perturbed_submap",
                           perturbed_reading_mesh_pub);
    // Publish the reading submap mesh used to indicate its optimized pose
    submap_vis.publishMesh(*submap_collection_ptr, reading_submap_id,
                           voxblox::Color::Blue(), "optimized_submap",
                           optimized_reading_mesh_pub);
  }

  // Finish the submaps such that their cached members are generated
  {
    VoxgraphSubmap::Ptr ref_submap_ptr =
        submap_collection_ptr->getSubmapPtr(reference_submap_id);
    VoxgraphSubmap::Ptr reading_submap_ptr =
        submap_collection_ptr->getSubmapPtr(reading_submap_id);
    CHECK_NOTNULL(ref_submap_ptr);
    CHECK_NOTNULL(reading_submap_ptr);
    ref_submap_ptr->setRegistrationFilterConfig(registration_filter_config);
    reading_submap_ptr->setRegistrationFilterConfig(registration_filter_config);
    ref_submap_ptr->finishSubmap();
    reading_submap_ptr->finishSubmap();
  }

  // Create the log file with the current timestamp as its file name
  time_t raw_time = std::time(nullptr);
  struct tm time_struct;
  localtime_r(&raw_time, &time_struct);
  char time_char_buffer[80];
  strftime(time_char_buffer, 80, "%Y-%m-%d_%H-%M-%S", &time_struct);
  boost::filesystem::path log_file_path;
  log_file_path = log_folder_path;
  log_file_path /= time_char_buffer;
  log_file_path += ".csv";
  ROS_INFO_STREAM("Log file will be saved as: " << log_file_path);
  std::ofstream log_file;
  log_file.open(log_file_path.string());

  // Create log file and write header
  // TODO(victorr): Check if log_folder_path exists and create it if it doesn't
  log_file << "reference_submap_id, reading_submap_id, "
           << "visuals_enabled, using_esdf_distance, git_branch, git_hash\n"
           << reference_submap_id << ",";
  if (reading_submap_id == INT32_MAX) {
    log_file << reference_submap_id;
  } else {
    log_file << reading_submap_id;
  }
  log_file << ","
           << (registerer_options.registration.visualize_residuals ||
               registerer_options.registration.visualize_gradients)
           << "," << registerer_options.registration.use_esdf_distance << ","
           << GIT_BRANCH << "," << GIT_COMMIT_HASH << "\n"
           << "x_true, y_true, z_true, yaw_true, pitch_true, roll_true\n"
           << ground_truth_position.x() << "," << ground_truth_position.y()
           << "," << ground_truth_position.z() << ","
           << "0,0,0\n";
  log_file << "x_disturbance, y_disturbance, z_disturbance,"
           << "yaw_disturbance, pitch_disturbance, roll_disturbance, "
           << "x_error, y_error, z_error, yaw_error, pitch_error, roll_error, "
           << "solve_time \n";

  // Loop over all ranges
  int counter(0);
  ROS_INFO("Looping over all starting positions:");
  for (const float& x : range_x) {
    for (const float& y : range_y)
      for (const float& z : range_z)
        for (const float& yaw : range_yaw)
          for (const float& pitch : range_pitch)
            for (const float& roll : range_roll) {
              // Append test to log file
              log_file << x << "," << y << "," << z << "," << yaw << ","
                       << pitch << "," << roll << ",";

              // Move reading_submap to T(x,y,z,yaw,pitch,roll)
              // TODO(victorr): Implement disturbance over pitch & roll
              Transformation::Vector3 rot_vec(0, 0, yaw);
              voxblox::Rotation rotation(rot_vec);
              Transformation T_mission__reading_perturbed;
              T_mission__reading_perturbed.getRotation() =
                  T_mission__reading_original.getRotation() * rotation;
              Transformation::Vector3 translation(x, y, z);
              T_mission__reading_perturbed.getPosition() =
                  T_mission__reading_original.getPosition() + translation;
              submap_collection_ptr->setSubmapPose(
                  reading_submap_id, T_mission__reading_perturbed);

              // Announce progress
              const Eigen::Vector3f& perturbed_position =
                  T_mission__reading_perturbed.getPosition();
              printf(
                  "-- % 2i disturbance:        "
                  "x % 4.6f    y % 4.6f    z % 4.6f    "
                  "yaw % 4.2f    pitch % 4.2f    roll % 4.2f\n",
                  counter, perturbed_position.x() - ground_truth_position.x(),
                  perturbed_position.y() - ground_truth_position.y(),
                  perturbed_position.z() - ground_truth_position.z(), yaw,
                  pitch, roll);

              // Publish the TF of perturbed mesh
              TfHelper::publishTransform(T_mission__reading_perturbed,
                                         "mission", "perturbed_submap", true);

              // Set initial conditions
              ceres::Solver::Summary summary;
              Transformation::Vector6 T_vec_read =
                  T_mission__reading_perturbed.log();
              double mission_pose_reading[4] = {T_vec_read[0], T_vec_read[1],
                                                T_vec_read[2], T_vec_read[5]};

              // Optimize submap registration
              bool registration_successful = submap_registerer.testRegistration(
                  reference_submap_id, reading_submap_id, mission_pose_reading,
                  &summary);
              if (registration_successful) {
                // Update reading submap pose with the optimization result
                T_vec_read[0] = mission_pose_reading[0];
                T_vec_read[1] = mission_pose_reading[1];
                T_vec_read[2] = mission_pose_reading[2];
                T_vec_read[5] = mission_pose_reading[3];
                const Transformation T_mission__reading_optimized =
                    Transformation::exp(T_vec_read);
                submap_collection_ptr->setSubmapPose(
                    reading_submap_id, T_mission__reading_optimized);

                // Announce results
                const Eigen::Vector3f& optimized_position =
                    T_mission__reading_optimized.getPosition();
                printf(
                    "-- % 2i remaining error:    "
                    "x % 4.6f    y % 4.6f    z % 4.6f    "
                    "yaw % 4.2f    pitch % 4.2f    roll % 4.2f    "
                    "time % 4.4f\n",
                    counter++,
                    optimized_position.x() - ground_truth_position.x(),
                    optimized_position.y() - ground_truth_position.y(),
                    optimized_position.z() - ground_truth_position.z(),
                    T_mission__reading_optimized.log()[5] -
                        T_mission__reading_original.log()[5],
                    0.0, 0.0, summary.total_time_in_seconds);

                // Append stats to log file
                log_file << optimized_position.x() - ground_truth_position.x()
                         << ","
                         << optimized_position.y() - ground_truth_position.y()
                         << ","
                         << optimized_position.z() - ground_truth_position.z()
                         << ",0,0,0," << summary.total_time_in_seconds << "\n";
              } else {
                ROS_WARN("Ceres could not find a solution");

                // Append failure to log file
                log_file << "X,X,X,X,X,X\n";
              }

              // Report solver stats
              if (solver_report_style == kBrief) {
                std::cout << summary.BriefReport() << std::endl;
              } else if (solver_report_style == kFull) {
                std::cout << summary.FullReport() << std::endl;
              }

              // Exit if CTRL+C was pressed
              if (!ros::ok()) {
                ROS_INFO("Shutting down...");
                goto endloop;
              }
            }
  }
endloop:

  // Close the log file
  log_file.close();

  // Keep the ROS node alive in order to interact with its topics in Rviz
  ros::spin();

  // Exit normally
  return 0;
}
