#include "voxgraph/tools/visualization/loop_closure_visuals.h"

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>

#include <eigen_conversions/eigen_msg.h>
#include <minkindr_conversions/kindr_msg.h>

namespace voxgraph {

void LoopClosureVisuals::publishLink(const Transformation& T_W_1,
                                     const Transformation& T_W_2,
                                     const std::string& frame_id,
                                     const ros::Publisher& publisher) const {
  // Creating the marker
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.05;
  marker.scale.y = 0.0;
  marker.scale.z = 0.0;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.ns = "link";
  // Extracting the matching frame end points
  // Adding the marker msg
  geometry_msgs::Point T_W_1_point_msg;
  geometry_msgs::Point T_W_2_point_msg;
  tf::pointEigenToMsg(T_W_1.getPosition().cast<double>(), T_W_1_point_msg);
  tf::pointEigenToMsg(T_W_2.getPosition().cast<double>(), T_W_2_point_msg);
  marker.points.push_back(T_W_1_point_msg);
  marker.points.push_back(T_W_2_point_msg);
  // Publish
  publisher.publish(marker);
}

void LoopClosureVisuals::publishAxes(const Transformation& T_W_1,
                                     const Transformation& T_W_2,
                                     const std::string& frame_id,
                                     const ros::Publisher& publisher) const {
  geometry_msgs::PoseArray pose_array_msg;
  pose_array_msg.header.frame_id = frame_id;
  geometry_msgs::Pose T_W_1_msg;
  geometry_msgs::Pose T_W_2_msg;
  tf::poseKindrToMsg(T_W_1.cast<double>(), &T_W_1_msg);
  tf::poseKindrToMsg(T_W_2.cast<double>(), &T_W_2_msg);
  pose_array_msg.poses.push_back(std::move(T_W_1_msg));
  pose_array_msg.poses.push_back(std::move(T_W_2_msg));
  publisher.publish(pose_array_msg);
}

}  // namespace voxgraph