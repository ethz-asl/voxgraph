#include "voxgraph/tools/data_servers/loop_closure_edge_server.h"
#include <voxgraph/common.h>

namespace voxgraph {
LoopClosureEdgeServer::LoopClosureEdgeServer(ros::NodeHandle nh_private) {
  loop_closure_edge_list_pub_ =
      nh_private.advertise<voxgraph_msgs::LoopClosureEdgeList>(
          "loop_closure_edges", 3, false);
}

void LoopClosureEdgeServer::publishLoopClosureEdges(
    const PoseGraphInterface &pose_graph_interface,
    const VoxgraphSubmapCollection &submap_collection,
    const ros::Time &timestamp) {
  // Only publish if there are subscribers
  if (loop_closure_edge_list_pub_.getNumSubscribers() > 0) {
    publishLoopClosureEdges(pose_graph_interface, submap_collection, timestamp,
                            loop_closure_edge_list_pub_);
  }
}

void LoopClosureEdgeServer::publishLoopClosureEdges(
    const PoseGraphInterface &pose_graph_interface,
    const VoxgraphSubmapCollection &submap_collection,
    const ros::Time &timestamp,
    const ros::Publisher &loop_closure_edge_list_publisher) {
  voxgraph_msgs::LoopClosureEdgeList loop_closure_edge_list_msg;
  loop_closure_edge_list_msg.header = generateHeaderMsg(timestamp);

  PoseGraphInterface::OverlappingSubmapList overlapping_submap_list =
      pose_graph_interface.getOverlappingSubmapList();
  loop_closure_edge_list_msg.loop_closure_edges.reserve(
      overlapping_submap_list.size());

  for (const SubmapIdPair &overlapping_submap_pair : overlapping_submap_list) {
    const VoxgraphSubmap &first_submap =
        submap_collection.getSubmap(overlapping_submap_pair.first);
    const VoxgraphSubmap &second_submap =
        submap_collection.getSubmap(overlapping_submap_pair.second);

    voxgraph_msgs::LoopClosureEdge edge_msg;
    edge_msg.timestamp_A = first_submap.getStartTime();
    edge_msg.timestamp_B = second_submap.getStartTime();

    Transformation T_A_B =
        first_submap.getPose().inverse() * second_submap.getPose();
    tf::poseKindrToMsg(T_A_B.cast<double>(), &edge_msg.T_A_B.pose);
    loop_closure_edge_list_msg.loop_closure_edges.push_back(edge_msg);
  }

  loop_closure_edge_list_publisher.publish(loop_closure_edge_list_msg);
}

std_msgs::Header LoopClosureEdgeServer::generateHeaderMsg(
    const ros::Time &timestamp) {
  std_msgs::Header msg_header;
  msg_header.frame_id = "";
  msg_header.stamp = timestamp;
  return msg_header;
}
}  // namespace voxgraph
