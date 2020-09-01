#ifndef VOXGRAPH_TOOLS_DATA_SERVERS_LOOP_CLOSURE_EDGE_SERVER_H_
#define VOXGRAPH_TOOLS_DATA_SERVERS_LOOP_CLOSURE_EDGE_SERVER_H_

#include <voxgraph/frontend/pose_graph_interface/pose_graph_interface.h>
#include <voxgraph_msgs/LoopClosureEdgeList.h>

#include "voxgraph/frontend/submap_collection/voxgraph_submap_collection.h"

namespace voxgraph {
class LoopClosureEdgeServer {
 public:
  explicit LoopClosureEdgeServer(ros::NodeHandle nh_private,
                                 bool verbose = false);

  // Publish the map using this map server instance's ros publisher member
  void publishLoopClosureEdges(
      const PoseGraphInterface& pose_graph_interface,
      const VoxgraphSubmapCollection& submap_collection,
      const ros::Time& timestamp);

  // "Bring your own publisher" method
  // NOTE: This method is provided s.t. it can be called using publishers to
  //       custom topics and without requiring a LoopClosureEdgeServer instance.
  //       It is therefore static.
  static void publishLoopClosureEdges(
      const PoseGraphInterface& pose_graph_interface,
      const VoxgraphSubmapCollection& submap_collection,
      const ros::Time& timestamp,
      const ros::Publisher& loop_closure_edge_list_publisher,
      bool verbose = false);

 private:
  bool verbose_;
  static constexpr bool kFake6dofTransforms = true;
  static constexpr double kSetUnknownCovarianceEntriesTo = 1e4;

  ros::Publisher loop_closure_edge_list_pub_;

  // Convenience methods to generate the message header
  static std_msgs::Header generateHeaderMsg(const ros::Time& timestamp);
};
}  // namespace voxgraph

#endif  // VOXGRAPH_TOOLS_DATA_SERVERS_LOOP_CLOSURE_EDGE_SERVER_H_
