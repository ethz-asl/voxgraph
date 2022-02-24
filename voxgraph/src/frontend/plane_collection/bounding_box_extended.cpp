#include "voxgraph/frontend/plane_collection/bounding_box_extended.h"

#include <algorithm>
#include <utility>

namespace voxgraph {

void BoundingBoxExtended::updateBoundingBoxLimits(const Point& p) {
  min.x() = std::min(p.x(), min.x());
  max.x() = std::max(p.x(), max.x());
  min.y() = std::min(p.y(), min.y());
  max.y() = std::max(p.y(), max.y());
  min.z() = std::min(p.z(), min.z());
  max.z() = std::max(p.z(), max.z());
}
/**
 * @brief Computes the intersection over Union between this box
 * and another one
 * @param other_bounding_box the other bounding box
 * @return float IoU metric
 */
float BoundingBoxExtended::IoU(
    voxgraph::BoundingBox* other_bounding_box) const {
  float union1 = computeVolume(max, min);
  float union2 =
      computeVolume(other_bounding_box->max, other_bounding_box->min);
  float xA = std::max(min.x(), other_bounding_box->min.x());
  float yA = std::max(min.y(), other_bounding_box->max.y());
  float zA = std::max(min.z(), other_bounding_box->max.z());
  float xB = std::min(max.x(), other_bounding_box->max.x());
  float yB = std::min(max.y(), other_bounding_box->max.y());
  float zB = std::min(max.z(), other_bounding_box->max.z());
  float interVolume =
      max(0, xB - xA + 1) * max(0, yB - yA + 1) * max(0, zB - zA + 1);
  float total_union = union1 + union2;
  assert(interVolume >= 0.0);
  assert(total_union >= 0.0);
  if (total_union == 0) {
    return 0.0;
  }
  return interVolume / total_union;
}

BoundingBoxMsg BoundingBoxExtended::toBoundingBoxMsg() const {
  BoundingBoxMsg ret;
  tf::pointEigenToMsg(max.cast<double>(), ret.max);
  tf::pointEigenToMsg(min.cast<double>(), ret.min);
  return std::move(ret);
}

BoundingBoxExtended BoundingBoxExtended::fromMsg(const BoundingBoxMsg& msg) {
  BoundingBoxExtended be;
  Eigen::Vector3d max_d, min_d;
  tf::pointMsgToEigen(msg.max, max_d);
  tf::pointMsgToEigen(msg.min, min_d);
  be.max = max_d.cast<float>();
  be.min = min_d.cast<float>();
  return be;
}

visualization_msgs::Marker BoundingBoxExtended::getVisualizationMsg() const {
  static const int bbox_indices[] = {0, 1, 2, 3, 0, 4, 5, 6, 7,
                                     3, 2, 6, 5, 4, 1, 4, 7};
  visualization_msgs::Marker ret;
  for (int i : bbox_indices) {
    Point p_f;
    p_f.x() = (!(i & 0b1)) ? min.x() : max.x();
    p_f.y() = (!(i & 0b10)) ? min.y() : max.y();
    p_f.z() = (!(i & 0b100)) ? min.z() : max.z();
    geometry_msgs::Point p_m;
    tf::pointEigenToMsg(p_f.cast<double>(), p_m);
    ret.points.push_back(p_m);
  }
  ret.header.frame_id = "world";
  ret.type = visualization_msgs::Marker::LINE_STRIP;
  ret.action = visualization_msgs::Marker::ADD;
  ret.pose.orientation.w = 1.0;
  ret.scale.x = 0.1;
  ret.color.b = 1.0;
  ret.color.a = 1.0;
  ret.header.stamp = ros::Time::now();
  return ret;
}

}  // namespace voxgraph
