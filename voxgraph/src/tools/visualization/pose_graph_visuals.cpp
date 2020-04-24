#include "voxgraph/tools/visualization/pose_graph_visuals.h"

#include <limits>
#include <string>
#include <vector>

namespace voxgraph {
void PoseGraphVisuals::publishPoseGraph(const PoseGraph& pose_graph,
                                        const std::string& frame_id,
                                        const std::string& name_space,
                                        const ros::Publisher& publisher) const {
  // Initialize the Rviz marker msg
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time();
  marker.ns = name_space;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;  // Set to unit quaternion
  marker.scale.x = 0.1;
  marker.frame_locked = false;

  // Get the costs and endpoints for all pose graph edges
  std::vector<PoseGraph::VisualizationEdge> edges =
      pose_graph.getVisualizationEdges();

  // Assert that the 'infinity' numeric limit is available at compile time,
  // such that it can be used below
  static_assert(std::numeric_limits<double>::has_infinity,
                "Support for "
                "std::numeric_limits<double>::infinity() is required");

  // Compute the maximum cost, for normalization of the edge colors
  double max_cost = -std::numeric_limits<double>::infinity();
  for (const PoseGraph::VisualizationEdge& edge : edges) {
    if (edge.residual > max_cost) {
      max_cost = edge.residual;
    }
  }

  // Add the edges to the marker
  for (const PoseGraph::VisualizationEdge& edge : edges) {
    // Add edge endpoints
    geometry_msgs::Point point_msg;
    tf::pointEigenToMsg(edge.first_node_position.cast<double>(), point_msg);
    marker.points.push_back(point_msg);
    tf::pointEigenToMsg(edge.second_node_position.cast<double>(), point_msg);
    marker.points.push_back(point_msg);

    // Color edge according to residual
    std_msgs::ColorRGBA color_msg;
    color_msg.a = 1;
    color_msg.r = static_cast<float>(edge.residual / max_cost);
    //    color_msg.r = float(std::log10(edge.residual + 1));
    marker.colors.push_back(color_msg);
    marker.colors.push_back(color_msg);
  }

  // Publish the marker
  publisher.publish(marker);
}
}  // namespace voxgraph
