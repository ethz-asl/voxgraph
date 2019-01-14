//
// Created by victor on 14.01.19.
//

//
// Created by victor on 29.11.18.
//

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

  // Generate the ESDFs for the submaps
  std::vector<cblox::SubmapID> submap_ids = submap_collection_ptr->getIDs();
  for (const cblox::SubmapID& submap_id : submap_ids) {
    CHECK(submap_collection_ptr->generateEsdfById(submap_id));
  }

  // TODO(victorr): Optimize the graph

  // Keep the ROS node alive in order to interact with its topics in Rviz
  ros::spin();

  return 0;
}
