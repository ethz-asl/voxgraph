//
// Created by victor on 14.01.19.
//

#include "voxgraph/pose_graph/pose_graph.h"
#include <cblox/core/submap_collection.h>
#include <cblox/io/tsdf_submap_io.h>
#include <glog/logging.h>
#include <ros/ros.h>
#include <string>
#include <vector>
#include "voxgraph/voxgraph_submap.h"

int main(int argc, char** argv) {
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
  cblox::SubmapCollection<voxgraph::VoxgraphSubmap>::Ptr submap_collection_ptr;
  cblox::io::LoadSubmapCollection<voxgraph::VoxgraphSubmap>(
      submap_collection_file_path, &submap_collection_ptr);

  // Create the pose graph
  voxgraph::PoseGraph pose_graph;

  // Generate the ESDFs for the submaps
  std::vector<cblox::SubmapID> submap_ids = submap_collection_ptr->getIDs();
  for (const cblox::SubmapID& submap_id : submap_ids) {
    CHECK(submap_collection_ptr->generateEsdfById(submap_id));
  }

  // Add all submaps as nodes
  unsigned int node_id = 0;
  for (const cblox::SubmapID& submap_id : submap_ids) {
    voxblox::Transformation node_pose;
    CHECK(submap_collection_ptr->getSubMapPose(submap_id, &node_pose));
    voxgraph::Node new_node = {node_id, submap_id, node_pose};
    pose_graph.addNode(new_node);
  }

  // Iterate over all submap pairs and check whether they overlap
  // For each overlapping pair, one constraint is added to the graph
  for (unsigned int i = 0; i < submap_ids.size(); i++) {
    // Get a pointer to the first submap
    cblox::SubmapID first_submap_id = submap_ids[i];
    voxgraph::VoxgraphSubmap::ConstPtr first_submap_ptr =
        submap_collection_ptr->getSubMapConstPtrById(first_submap_id);
    CHECK_NOTNULL(first_submap_ptr);

    for (unsigned int j = i + 1; j < submap_ids.size(); j++) {
      // Get a pointer to the second submap
      cblox::SubmapID second_submap_id = submap_ids[j];
      voxgraph::VoxgraphSubmap::ConstPtr second_submap_ptr =
          submap_collection_ptr->getSubMapConstPtrById(second_submap_id);
      CHECK_NOTNULL(second_submap_ptr);

      // Check whether the first and second submap overlap
      bool overlap = first_submap_ptr->overlapsWith(second_submap_ptr);
      if (overlap) {
        // Add the constraint
        voxgraph::Constraint new_constraint = {first_submap_id,
                                               second_submap_id};
        pose_graph.addConstraint(new_constraint);
      }
    }
  }

  // Optimize the graph
  pose_graph.solve();

  // TODO(victorr): Update the submap poses

  // Keep the ROS node alive in order to interact with its topics in Rviz
  ros::spin();

  return 0;
}
