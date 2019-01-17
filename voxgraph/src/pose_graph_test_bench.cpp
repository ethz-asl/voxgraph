//
// Created by victor on 14.01.19.
//

#include "voxgraph/pose_graph/pose_graph.h"
#include <cblox/core/submap_collection.h>
#include <cblox/io/tsdf_submap_io.h>
#include <glog/logging.h>
#include <ros/ros.h>
#include <memory>
#include <string>
#include <vector>
#include "voxgraph/visualization/submap_visuals.h"
#include "voxgraph/voxgraph_submap.h"

int main(int argc, char** argv) {
  using voxgraph::VoxgraphSubmap;
  using voxgraph::PoseGraph;
  using voxgraph::SubmapNode;
  using voxgraph::RegistrationConstraint;
  using voxgraph::SubmapVisuals;

  // Start logging
  google::InitGoogleLogging(argv[0]);

  // Register with ROS master
  ros::init(argc, argv, "voxgraph_pose_graph_test_bench");

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

  // Load the submap collection
  cblox::SubmapCollection<VoxgraphSubmap>::Ptr submap_collection_ptr;
  cblox::io::LoadSubmapCollection<VoxgraphSubmap>(submap_collection_file_path,
                                                  &submap_collection_ptr);

  // Create the pose graph
  PoseGraph pose_graph(submap_collection_ptr);

  // Generate the ESDFs for the submaps
  std::vector<cblox::SubmapID> submap_ids = submap_collection_ptr->getIDs();
  for (const cblox::SubmapID& submap_id : submap_ids) {
    CHECK(submap_collection_ptr->generateEsdfById(submap_id));
  }

  // Setup Rviz visualizations
  VoxgraphSubmap::Config submap_config;
  {
    // Configure the submap_config
    VoxgraphSubmap::ConstPtr submap_ptr =
        submap_collection_ptr->getSubMapConstPtrById(submap_ids[0]);
    CHECK_NOTNULL(submap_ptr);
    submap_config.tsdf_voxel_size =
        submap_ptr->getTsdfMap().getTsdfLayer().voxel_size();
    submap_config.tsdf_voxels_per_side =
        size_t(submap_ptr->getTsdfMap().getTsdfLayer().block_size() /
               submap_config.tsdf_voxel_size);
  }
  SubmapVisuals submap_vis(submap_config);
  ros::Publisher separated_mesh_original_pub =
      nh_private.advertise<visualization_msgs::Marker>(
          "separated_mesh_original", 1);
  ros::Publisher separated_mesh_optimized_pub =
      nh_private.advertise<visualization_msgs::Marker>(
          "separated_mesh_optimized", 1);

  // Show the original submap meshes in Rviz
  {
    // Wait for Rviz to launch so that it receives the meshes
    ros::Rate wait_rate(1);
    while (separated_mesh_original_pub.getNumSubscribers() == 0) {
      std::cout << "Waiting for Rviz to launch and subscribe "
                << "to topic 'separated_mesh_original'" << std::endl;
      wait_rate.sleep();
    }
  }
  submap_vis.publishSeparatedMesh(*submap_collection_ptr, "world",
                                  separated_mesh_original_pub);

  // Add all submaps as nodes
  std::cout << "Adding all submaps as nodes" << std::endl;
  for (const cblox::SubmapID& submap_id : submap_ids) {
    voxblox::Transformation initial_pose;
    CHECK(submap_collection_ptr->getSubMapPose(submap_id, &initial_pose));
    SubmapNode::Config node_config = {submap_id, initial_pose};
    pose_graph.addNode(node_config);
  }

  // Add odometry constraints between the submaps
  for (const cblox::SubmapID& submap_id : submap_ids) {
    // TODO(victorr): Implement odometry constraints
  }

  // Add a registration constraint for each overlapping submap pair
  std::cout << "Adding registration constraint from submap " << std::endl;
  for (unsigned int i = 0; i < submap_ids.size(); i++) {
    // Get a pointer to the first submap
    cblox::SubmapID first_submap_id = submap_ids[i];
    VoxgraphSubmap::ConstPtr first_submap_ptr =
        submap_collection_ptr->getSubMapConstPtrById(first_submap_id);
    CHECK_NOTNULL(first_submap_ptr);

    for (unsigned int j = i + 1; j < submap_ids.size(); j++) {
      // Get a pointer to the second submap
      cblox::SubmapID second_submap_id = submap_ids[j];
      VoxgraphSubmap::ConstPtr second_submap_ptr =
          submap_collection_ptr->getSubMapConstPtrById(second_submap_id);
      CHECK_NOTNULL(second_submap_ptr);

      // Check whether the first and second submap overlap
      if (first_submap_ptr->overlapsWith(second_submap_ptr)) {
        std::cout << "--  " << first_submap_id << " to " << second_submap_id
                  << std::endl;
        // Add the constraint
        RegistrationConstraint::Config constraint_config = {first_submap_id,
                                                            second_submap_id};
        pose_graph.addConstraint(constraint_config);
      }
    }
  }

  // Optimize the graph
  std::cout << "Optimizing the graph" << std::endl;
  pose_graph.optimize();

  // Update the submap poses
  for (const auto& submap_pose_kv : pose_graph.getSubmapPoses()) {
    voxblox::Transformation original_pose;
    submap_collection_ptr->getSubMapPose(submap_pose_kv.first, &original_pose);
    std::cout << "Updating submap " << submap_pose_kv.first << " from \n"
              << original_pose.getPosition() << "\n to \n"
              << submap_pose_kv.second.getPosition() << std::endl;
    submap_collection_ptr->setSubMapPose(submap_pose_kv.first,
                                         submap_pose_kv.second);
  }

  // Show the optimized submap meshes in Rviz
  submap_vis.publishSeparatedMesh(*submap_collection_ptr, "world",
                                  separated_mesh_optimized_pub);

  // Keep the ROS node alive in order to interact with its topics in Rviz
  std::cout << "Done" << std::endl;
  ros::spin();

  return 0;
}
