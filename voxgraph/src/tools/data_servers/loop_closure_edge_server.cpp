#include "voxgraph/tools/data_servers/loop_closure_edge_server.h"

#include <limits>

#include "voxgraph/common.h"

namespace voxgraph {

constexpr bool LoopClosureEdgeServer::kFake6dofTransforms;
constexpr double LoopClosureEdgeServer::kSetUnknownCovarianceEntriesTo;

LoopClosureEdgeServer::LoopClosureEdgeServer(ros::NodeHandle nh_private,
                                             bool verbose)
    : verbose_(verbose) {
  loop_closure_edge_list_pub_ =
      nh_private.advertise<voxgraph_msgs::LoopClosureEdgeList>(
          "loop_closure_edges", 3, false);
}

void LoopClosureEdgeServer::publishLoopClosureEdges(
    const PoseGraphInterface& pose_graph_interface,
    const VoxgraphSubmapCollection& submap_collection,
    const ros::Time& timestamp) {
  // Only publish if there are subscribers
  if (loop_closure_edge_list_pub_.getNumSubscribers() > 0) {
    publishLoopClosureEdges(pose_graph_interface, submap_collection, timestamp,
                            loop_closure_edge_list_pub_, verbose_);
  }
}

void LoopClosureEdgeServer::publishLoopClosureEdges(
    const PoseGraphInterface& pose_graph_interface,
    const VoxgraphSubmapCollection& submap_collection,
    const ros::Time& timestamp,
    const ros::Publisher& loop_closure_edge_list_publisher, bool verbose) {
  voxgraph_msgs::LoopClosureEdgeList loop_closure_edge_list_msg;
  loop_closure_edge_list_msg.header = generateHeaderMsg(timestamp);

  PoseGraphInterface::OverlappingSubmapList overlapping_submap_list =
      pose_graph_interface.getOverlappingSubmapList();
  loop_closure_edge_list_msg.loop_closure_edges.reserve(
      overlapping_submap_list.size());

  PoseGraph::EdgeCovarianceMap edge_covariance_map;
  bool edge_covariances_usable =
      pose_graph_interface.getEdgeCovarianceMap(&edge_covariance_map);

  for (const SubmapIdPair& overlapping_submap_pair : overlapping_submap_list) {
    // Get const refs to the submap at each endpoint of the edge
    const VoxgraphSubmap& first_submap =
        submap_collection.getSubmap(overlapping_submap_pair.first);
    const VoxgraphSubmap& second_submap =
        submap_collection.getSubmap(overlapping_submap_pair.second);

    // Create the loop closure edge msg
    voxgraph_msgs::LoopClosureEdge edge_msg;
    edge_msg.timestamp_A = first_submap.getStartTime();
    edge_msg.timestamp_B = second_submap.getStartTime();

    // Set the edge's observed transformation
    Transformation T_O_A = first_submap.getPose();
    Transformation T_O_B = second_submap.getPose();

    if (LoopClosureEdgeServer::kFake6dofTransforms) {
      // Go from 4DoF submap origin poses T_O_S to 6DoF robot poses T_O_R
      // by using T_O_B = T_O_S * T_S_R, where T_S_R is the 6DoF robot pose at
      // submap creation time
      // NOTE: The loop closure edge covariances are estimated for the 4DoF
      //       submap poses. Using covariances in combination with 6DoF poses
      //       therefore isn't correct, but might be acceptably close for small
      //       pitch/roll.
      T_O_A = T_O_A * first_submap.getPoseHistory().begin()->second;
      T_O_B = T_O_B * second_submap.getPoseHistory().begin()->second;
    }

    Transformation T_A_B = T_O_A.inverse() * T_O_B;
    tf::poseKindrToMsg(T_A_B.cast<double>(), &edge_msg.T_A_B.pose);

    // Set the edge's covariance
    if (edge_covariances_usable) {
      // Get the covariance from the pose graph
      PoseGraph::EdgeCovarianceMap::const_iterator covariance_iter =
          edge_covariance_map.find(overlapping_submap_pair);
      if (covariance_iter != edge_covariance_map.end()) {
        edge_msg.T_A_B.covariance.fill(
            LoopClosureEdgeServer::kSetUnknownCovarianceEntriesTo);
        for (int original_row = 0; original_row < 4; ++original_row) {
          for (int original_col = 0; original_col < 4; ++original_col) {
            int msg_row = original_row;
            int msg_col = original_col;
            if (original_row == 3) msg_row = 5;
            if (original_col == 3) msg_col = 5;
            edge_msg.T_A_B.covariance[msg_row * 6 + msg_col] =
                covariance_iter->second(original_row, original_col);
          }
        }

        // Print the msg's covariance matrix for debugging
        if (verbose) {
          ROS_INFO_STREAM("Edge covariance from submap "
                          << overlapping_submap_pair.first << " to "
                          << overlapping_submap_pair.second << " is:\n");
          for (int row = 0; row < 6; ++row) {
            for (int col = 0; col < 6; ++col) {
              if (col == 0) {
                std::cout << std::endl;
              }
              std::cout << edge_msg.T_A_B.covariance[row * 6 + col] << "   ";
            }
          }
          std::cout << std::endl;
        }
      } else {
        ROS_WARN(
            "Missing edge covariance for submap pair despite successful "
            "covariance extraction. This should not happen. The loop "
            "closure edge list will not be published.");
        return;
      }
    } else {
      // Set the covariance to identity
      for (int row = 0; row < 6; ++row) {
        for (int col = 0; col < 6; ++col) {
          if (row == col) {
            edge_msg.T_A_B.covariance[row * 6 + col] = 1.0;
          } else {
            edge_msg.T_A_B.covariance[row * 6 + col] = 0.0;
          }
        }
      }
    }

    // Add the edge msg to the list
    loop_closure_edge_list_msg.loop_closure_edges.push_back(edge_msg);
  }

  // Publish the loop closure edge list msg
  loop_closure_edge_list_publisher.publish(loop_closure_edge_list_msg);
}

std_msgs::Header LoopClosureEdgeServer::generateHeaderMsg(
    const ros::Time& timestamp) {
  std_msgs::Header msg_header;
  msg_header.frame_id = "";
  msg_header.stamp = timestamp;
  return msg_header;
}
}  // namespace voxgraph
