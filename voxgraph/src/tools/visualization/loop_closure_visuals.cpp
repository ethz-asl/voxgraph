#include "voxgraph/tools/visualization/loop_closure_visuals.h"

#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <eigen_conversions/eigen_msg.h>
#include <minkindr_conversions/kindr_msg.h>

namespace voxgraph {

void LoopClosureVisuals::publishLoopClosure(
    const Transformation& T_W_1, const Transformation& T_W_2,
    const Transformation& T_1_2_est, const std::string& frame_id,
    const ros::Publisher& publisher) const {
  visualization_msgs::MarkerArray marker_array;
  // Getting links
  static const Color kColorOld = Color(0.0f, 0.0f, 1.0f, 1.0f);
  static const Color kColorEst = Color(1.0f, 0.0f, 0.0f, 1.0f);
  visualization_msgs::Marker old_link =
      getLinkMarker(T_W_1, T_W_2, frame_id, kColorOld, "old_link");
  const Transformation T_W_2_est = T_W_1 * T_1_2_est;
  visualization_msgs::Marker new_link =
      getLinkMarker(T_W_1, T_W_2_est, frame_id, kColorEst, "new_link");
  // Publishing
  marker_array.markers.push_back(old_link);
  marker_array.markers.push_back(new_link);
  publisher.publish(marker_array);
}

visualization_msgs::Marker LoopClosureVisuals::getLinkMarker(
    const Transformation& T_W_1, const Transformation& T_W_2,
    const std::string& frame_id, const Color& color,
    const std::string& ns) const {
  // Creating the marker
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.05;
  marker.scale.y = 0.0;
  marker.scale.z = 0.0;
  marker.color.a = 1.0;
  marker.color.r = color.r;
  marker.color.g = color.g;
  marker.color.b = color.b;
  marker.ns = ns;
  // Extracting the matching frame end points
  // Adding the marker msg
  geometry_msgs::Point T_W_1_point_msg;
  geometry_msgs::Point T_W_2_point_msg;
  tf::pointEigenToMsg(T_W_1.getPosition().cast<double>(), T_W_1_point_msg);
  tf::pointEigenToMsg(T_W_2.getPosition().cast<double>(), T_W_2_point_msg);
  marker.points.push_back(T_W_1_point_msg);
  marker.points.push_back(T_W_2_point_msg);
  return marker;
}

void LoopClosureVisuals::publishAxes(const Transformation& T_W_1,
                                     const Transformation& T_W_2,
                                     const Transformation& T_1_2_est,
                                     const std::string& frame_id,
                                     const ros::Publisher& publisher) const {
  // Calculating the estimate transform
  const Transformation T_W_2_est = T_W_1 * T_1_2_est;
  // Publish
  geometry_msgs::PoseArray pose_array_msg;
  pose_array_msg.header.frame_id = frame_id;
  geometry_msgs::Pose T_W_1_msg;
  geometry_msgs::Pose T_W_2_msg;
  geometry_msgs::Pose T_W_2_est_msg;
  tf::poseKindrToMsg(T_W_1.cast<double>(), &T_W_1_msg);
  tf::poseKindrToMsg(T_W_2.cast<double>(), &T_W_2_msg);
  tf::poseKindrToMsg(T_W_2_est.cast<double>(), &T_W_2_est_msg);
  pose_array_msg.poses.push_back(std::move(T_W_1_msg));
  pose_array_msg.poses.push_back(std::move(T_W_2_msg));
  pose_array_msg.poses.push_back(std::move(T_W_2_est_msg));
  publisher.publish(pose_array_msg);
}

}  // namespace voxgraph