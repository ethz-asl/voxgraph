//
// Created by victor on 29.11.18.
//

#include <cblox/core/submap_collection.h>
#include <cblox/io/tsdf_submap_io.h>
#include <cblox/utils/quat_transformation_protobuf_utils.h>
#include <glog/logging.h>
#include <ros/ros.h>
#include <time.h>
#include <voxblox/io/layer_io.h>
#include <voxblox_ros/ptcloud_vis.h>
#include <voxgraph/voxgraph_submap.h>
#include <boost/filesystem.hpp>
#include <string>
#include <vector>
#include "voxgraph/submap_registration/submap_registerer.h"
#include "voxgraph/visualization/submap_visuals.h"
#include "voxgraph/visualization/tf_helper.h"

int main(int argc, char** argv) {
  // Start logging
  google::InitGoogleLogging(argv[0]);

  // Register with ROS master
  ros::init(argc, argv, "voxgraph_registration_test_bench");

  // Create node handles
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Read ROS params
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
    reference_submap_id = fixed_submap_id_tmp;
    reading_submap_id = reading_submap_id_tmp;
  }
  bool use_esdf_distance;
  bool visualize_residuals, visualize_gradients;
  std::vector<double> range_x, range_y, range_z;
  std::vector<double> range_yaw, range_pitch, range_roll;
  nh_private.param("range_x", range_x, {0});
  nh_private.param("range_y", range_y, {0});
  nh_private.param("range_z", range_z, {0});
  nh_private.param("range_yaw", range_yaw, {0});
  nh_private.param("range_pitch", range_pitch, {0});
  nh_private.param("range_roll", range_roll, {0});
  nh_private.param("use_esdf_distance", use_esdf_distance, true);
  nh_private.param("visualize_residuals", visualize_residuals, false);
  nh_private.param("visualize_gradients", visualize_gradients, false);

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

  // NOTE(victorr): The code below comes from cblox,
  // but has been modified to selectively load submaps.
  // TODO(victorr): Integrate the feature back into cblox.
  cblox::SubmapCollection<voxgraph::VoxgraphSubmap>::Ptr submap_collection_ptr;
  voxgraph::VoxgraphSubmap::Config submap_config;
  {
    // Load the protobuf file containing the submap collection
    std::fstream proto_file;
    proto_file.open(submap_collection_file_path, std::fstream::in);
    if (!proto_file.is_open()) {
      LOG(FATAL) << "Could not open protobuf file to load layer: "
                 << submap_collection_file_path;
    }
    uint32_t tmp_byte_offset = 0;  // Protobuf byte offset

    // Load the protobuf header
    cblox::TsdfSubmapCollectionProto tsdf_submap_collection_proto;
    if (!voxblox::utils::readProtoMsgFromStream(
            &proto_file, &tsdf_submap_collection_proto, &tmp_byte_offset)) {
      LOG(FATAL) << "Could not read submap collection protobuf message.";
    }

    // Debug info
    std::cout << "tsdf_submap_collection_proto.voxel_size(): "
              << tsdf_submap_collection_proto.voxel_size() << std::endl;
    std::cout << "tsdf_submap_collection_proto.voxels_per_side(): "
              << tsdf_submap_collection_proto.voxels_per_side() << std::endl;
    std::cout << "tsdf_submap_collection_proto.num_submaps(): "
              << tsdf_submap_collection_proto.num_submaps() << std::endl;

    // Creating the new submap collection based on the loaded parameters
    submap_config.tsdf_voxel_size = tsdf_submap_collection_proto.voxel_size();
    submap_config.tsdf_voxels_per_side =
        tsdf_submap_collection_proto.voxels_per_side();
    submap_config.esdf_voxel_size = submap_config.tsdf_voxel_size;
    submap_config.esdf_voxels_per_side = submap_config.tsdf_voxels_per_side;
    submap_collection_ptr.reset(
        new cblox::SubmapCollection<voxgraph::VoxgraphSubmap>(submap_config));

    // Load the two submaps of interest
    std::cout << "Searching for submap IDs " << reference_submap_id << " and "
              << reading_submap_id << " in protobuf:" << std::endl;
    for (size_t sub_map_index = 0;
         sub_map_index < tsdf_submap_collection_proto.num_submaps();
         sub_map_index++) {
      CHECK(submap_collection_ptr);

      // Getting the header for the current submap
      cblox::TsdfSubmapProto tsdf_sub_map_proto;
      if (!voxblox::utils::readProtoMsgFromStream(
              &proto_file, &tsdf_sub_map_proto, &tmp_byte_offset)) {
        LOG(ERROR) << "Could not read tsdf sub map protobuf message.";
        continue;
      }

      if (tsdf_sub_map_proto.id() == reference_submap_id) {
        std::cout << "-- Found submap corresponding to reference_submap_id: "
                  << reference_submap_id << std::endl;
      } else if (tsdf_sub_map_proto.id() == reading_submap_id) {
        std::cout << "-- Found submap corresponding to reading_submap_id: "
                  << reading_submap_id << std::endl;
      } else {
        std::cout << "-- Skipping submap with ID " << tsdf_sub_map_proto.id()
                  << std::endl;
        // Advance byte_offset in protobuf but don't add them to the map
        // TODO(victorr): Advancing the byte_offset directly would be nicer
        // but seems dangerous. Is there a better solution?
        cblox::TsdfMap tmp_tsdf_map(submap_config);
        voxblox::io::LoadBlocksFromStream(
            tsdf_sub_map_proto.num_blocks(),
            voxblox::Layer<voxblox::TsdfVoxel>::BlockMergingStrategy::kReplace,
            &proto_file, tmp_tsdf_map.getTsdfLayerPtr(), &tmp_byte_offset);
        continue;
      }

      // Get the transformation
      cblox::Transformation T_M_S;
      cblox::QuatTransformationProto transformation_proto =
          tsdf_sub_map_proto.transform();
      cblox::conversions::transformProtoToKindr(transformation_proto, &T_M_S);

      // Debug info
      std::cout << "--- Tsdf number of allocated blocks: "
                << tsdf_sub_map_proto.num_blocks() << std::endl;
      Eigen::Vector3 t = T_M_S.getPosition();
      cblox::Quaternion q = T_M_S.getRotation();
      std::cout << "--- [ " << t.x() << ", " << t.y() << ", " << t.z() << ", "
                << q.w() << q.x() << ", " << q.y() << ", " << q.z() << " ]"
                << std::endl;

      // Creating a new submap to hold the data
      submap_collection_ptr->createNewSubMap(T_M_S, tsdf_sub_map_proto.id());

      // Getting the blocks for this submap (the tsdf layer)
      if (!voxblox::io::LoadBlocksFromStream(
              tsdf_sub_map_proto.num_blocks(),
              voxblox::Layer<
                  voxblox::TsdfVoxel>::BlockMergingStrategy::kReplace,
              &proto_file,
              submap_collection_ptr->getActiveTsdfMapPtr()->getTsdfLayerPtr(),
              &tmp_byte_offset)) {
        LOG(ERROR) << "Could not load the blocks from stream.";
        continue;
      }
    }
    // Because grown ups clean up after themselves
    proto_file.close();
  }

  // If both submaps IDs are the same, duplicate the reference submap
  if (reference_submap_id == reading_submap_id) {
    std::cout << "Reference and reading submap IDs are the same, "
              << "duplicating the reference..." << std::endl;
    reading_submap_id = INT32_MAX;
    CHECK(submap_collection_ptr->duplicateSubMap(reference_submap_id,
                                                 reading_submap_id));
  }

  // Setup the submap to submap registerer
  // TODO(victorr): Set these parameters from ROS
  voxgraph::SubmapRegisterer::Options registerer_options;
  registerer_options.cost.cost_function_type =
      voxgraph::SubmapRegisterer::Options::CostFunction::Type::kAnalytic;
  registerer_options.cost.min_voxel_weight = 1e-6;
  registerer_options.cost.max_voxel_distance = 0.6;
  registerer_options.cost.no_correspondence_cost = 0;
  registerer_options.cost.use_esdf_distance = use_esdf_distance;
  registerer_options.cost.visualize_residuals = visualize_residuals;
  registerer_options.cost.visualize_gradients = visualize_gradients;
  registerer_options.solver.max_num_iterations = 40;
  registerer_options.solver.parameter_tolerance = 3e-3;
  registerer_options.param.optimize_yaw = true;
  voxgraph::SubmapRegisterer submap_registerer(submap_collection_ptr,
                                               registerer_options);

  // Setup visualization tools
  voxgraph::SubmapVisuals submap_vis(submap_config);

  // Save the reference submap pose and the original reading submap pose
  cblox::Transformation transform_getter;
  CHECK(submap_collection_ptr->getSubMapPose(reference_submap_id,
                                             &transform_getter));
  const cblox::Transformation T_world__reference = transform_getter;
  CHECK(submap_collection_ptr->getSubMapPose(reading_submap_id,
                                             &transform_getter));
  const cblox::Transformation T_world__reading_original = transform_getter;
  const Eigen::Vector3f &ground_truth_position =
      T_world__reading_original.getPosition();

  // Publish TFs for the reference and reading submap
  voxgraph::TfHelper::publishTransform(T_world__reference, "world",
                                       "reference_submap", true);
  voxgraph::TfHelper::publishTransform(T_world__reading_original, "world",
                                       "reading_submap", true);

  // Publish the meshes used to visualize the submaps
  {
    // Wait for Rviz to launch so that it receives the meshes
    ros::Rate wait_rate(1);
    while (reference_mesh_pub.getNumSubscribers() == 0) {
      std::cout << "Waiting for Rviz to launch "
                << "and subscribe to topic 'reference_mesh_pub'" << std::endl;
      wait_rate.sleep();
    }
    // Publish reference submap Mesh
    submap_vis.publishMesh(*submap_collection_ptr, reference_submap_id,
                           voxblox::Color::Green(), "reference_submap",
                           reference_mesh_pub);
    // Publish temporary TFs for the moving meshes, such that Rviz
    // doesn't discard them due to missing frame position information
    voxgraph::TfHelper::publishTransform(T_world__reading_original, "world",
                                         "perturbed_submap", true);
    voxgraph::TfHelper::publishTransform(T_world__reading_original, "world",
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

  // Generate the ESDFs for the submaps
  if (use_esdf_distance) {
    CHECK(submap_collection_ptr->generateEsdfById(reference_submap_id));
    CHECK(submap_collection_ptr->generateEsdfById(reading_submap_id));
  }

  // Format log file path containing current time stamp
  time_t raw_time = std::time(nullptr);
  struct tm time_struct;
  localtime_r(&raw_time, &time_struct);
  char time_char_buffer[80];
  strftime(time_char_buffer, 80, "%Y-%m-%d_%H-%M-%S", &time_struct);
  boost::filesystem::path log_file_path;
  log_file_path = log_folder_path;
  log_file_path /= time_char_buffer;
  log_file_path += ".csv";
  std::cout << "Log file will be saved as: " << log_file_path << std::endl;

  // Create log file and write header
  // TODO(victorr): Write Git ID into log file
  // TODO(victorr): Check if log_folder_path exists and create it if it doesn't
  std::ofstream log_file;
  log_file.open(log_file_path.string());
  log_file << "reference_submap_id, reading_submap_id, "
           << "visuals_enabled, using_esdf_distance\n"
           << reference_submap_id << ",";
  if (reading_submap_id == INT32_MAX) {
    log_file << reference_submap_id;
  } else {
    log_file << reading_submap_id;
  }
  log_file << "," << (visualize_residuals || visualize_gradients) << ","
           << use_esdf_distance << "\n"
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
  std::cout << "Looping over all starting positions:" << std::endl;
  for (auto x : range_x) {
    for (auto y : range_y)
      for (auto z : range_z)
        for (auto yaw : range_yaw)
          for (auto pitch : range_pitch)
            for (auto roll : range_roll) {
              // Append test to log file
              log_file << x << "," << y << "," << z << "," << yaw << ","
                       << pitch << "," << roll << ",";

              // Move reading_submap to T(x,y,z,yaw,pitch,roll)
              Eigen::Vector3d position(x, y, z);
              Eigen::Quaterniond orientation =
                  Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
                  Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
              cblox::Transformation perturbation(
                  position.cast<cblox::FloatingPoint>(),
                  orientation.cast<cblox::FloatingPoint>());
              const cblox::Transformation T_world__reading_perturbed =
                  perturbation * T_world__reading_original;
              submap_collection_ptr->setSubMapPose(reading_submap_id,
                                                   T_world__reading_perturbed);

              // Announce progress
              const Eigen::Vector3f &perturbed_position =
                  T_world__reading_perturbed.getPosition();
              printf(
                  "-- % 2i disturbance:        "
                  "x % 4.6f    y % 4.6f    z % 4.6f    "
                  "yaw % 4.2f    pitch % 4.2f    roll % 4.2f\n",
                  counter, perturbed_position.x() - ground_truth_position.x(),
                  perturbed_position.y() - ground_truth_position.y(),
                  perturbed_position.z() - ground_truth_position.z(), yaw,
                  pitch, roll);

              // Publish the TF of perturbed mesh
              voxgraph::TfHelper::publishTransform(T_world__reading_perturbed,
                                                   "world", "perturbed_submap",
                                                   true);

              // Set initial conditions
              ceres::Solver::Summary summary;
              voxblox::Transformation::Vector6 T_vec_read =
                  T_world__reading_perturbed.log();
              double world_pose_reading[4] = {T_vec_read[0], T_vec_read[1],
                                              T_vec_read[2], T_vec_read[5]};

              // Optimize submap registration
              bool registration_successful = submap_registerer.testRegistration(
                  reference_submap_id, reading_submap_id, world_pose_reading,
                  &summary);
              if (registration_successful) {
                // Update reading submap pose with the optimization result
                T_vec_read[0] = world_pose_reading[0];
                T_vec_read[1] = world_pose_reading[1];
                T_vec_read[2] = world_pose_reading[2];
                if (registerer_options.param.optimize_yaw) {
                  T_vec_read[5] = world_pose_reading[3];
                }
                const voxblox::Transformation T_world__reading_optimized =
                    voxblox::Transformation::exp(T_vec_read);
                submap_collection_ptr->setSubMapPose(
                    reading_submap_id, T_world__reading_optimized);

                // Announce results
                const Eigen::Vector3f optimized_position =
                    T_world__reading_optimized.getPosition();
                printf(
                    "-- % 2i remaining error:    "
                    "x % 4.6f    y % 4.6f    z % 4.6f    "
                    "yaw % 4.2f    pitch % 4.2f    roll % 4.2f    "
                    "time % 4.4f\n",
                    counter++,
                    optimized_position.x() - ground_truth_position.x(),
                    optimized_position.y() - ground_truth_position.y(),
                    optimized_position.z() - ground_truth_position.z(), 0.0,
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

              // TODO(victorr): Set report style from params
              std::cout << summary.BriefReport() << std::endl;
              //  std::cout << summary.FullReport() << std::endl;

              ros::spinOnce();
            }
  }

  // Close the log file and exit normally
  log_file.close();

  // Keep the ROS node alive in order to interact with its topics in Rviz
  ros::spin();

  return 0;
}
