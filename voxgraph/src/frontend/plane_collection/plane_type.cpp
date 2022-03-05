#include "voxgraph/frontend/plane_collection/plane_type.h"

#include <map>
#include <vector>

#include <minkindr_conversions/kindr_msg.h>
#include <panoptic_mapping_msgs/PlaneType.h>

namespace voxgraph {

PlaneType::PlaneType(const Eigen::Vector3f& normal, const Point& point,
                     const int class_id)
    : plane_(normal.stableNormalized(), normal.stableNormalized().dot(point)),
      point_(point),
      class_id_(class_id),
      plane_id_(getNextPlaneID()) {
  buildPlaneOrientation();
  T_M_P_init_ = T_M_P_;
  num_points_ = 0;
}

PlaneType::PlaneType(const Eigen::Hyperplane<float, 3>& plane, int class_id)
    : plane_(plane), class_id_(class_id), plane_id_(getNextPlaneID()) {
  const auto& n = plane.normal();
  point_ = plane.offset() * n;
  buildPlaneOrientation();
  T_M_P_init_ = T_M_P_;
  num_points_ = 0;
}

PlaneType::PlaneType(const Eigen::Hyperplane<float, 3>& plane,
                     const Transformation& T_M_P, int class_id)
    : plane_(plane), class_id_(class_id), plane_id_(getNextPlaneID()) {
  const auto& n = plane.normal();
  point_ = T_M_P.getPosition();
  T_M_P_ = T_M_P;
  T_M_P_init_ = T_M_P;
  num_points_ = 0;
}

PlaneType::PlaneType(const Eigen::Vector3f& normal, const Point& point,
                     const Transformation& T_M_P, int class_id)
    : PlaneType(Eigen::Hyperplane<float, 3>(normal.stableNormalized(), point), T_M_P, class_id) {
  // CHECK(normal.squaredNorm() - 1 < 1.01 &&
  //       1 - normal.squaredNorm() > 0.999);
  // normal check to be removed 
}

// getters
Eigen::Vector3f PlaneType::getPlaneNormal() const {
  return T_M_P_.getRotationMatrix().col(2);
}
Point PlaneType::getPointInit() const { return point_; }
Transformation PlaneType::getPlaneTransformation() const { return T_M_P_; }
PlaneType::PlaneID PlaneType::getPlaneID() const { return plane_id_; }
const voxgraph::BoundingBox* PlaneType::getBoundingBox() const {
  return &planeSegmentAaBb_;
}
visualization_msgs::Marker PlaneType::getVisualizationMsg() const {
  return planeSegmentAaBb_.getVisualizationMsg();
}
visualization_msgs::Marker PlaneType::getNormalVisualizationMsg() const {
  static int marker_id = 10000;
  visualization_msgs::Marker msg;
  msg.header.frame_id = "world";
  msg.type = visualization_msgs::Marker::LINE_LIST;
  msg.action = visualization_msgs::Marker::ADD;
  geometry_msgs::Point point_msg;
  geometry_msgs::Point vec_end_msg;
  tf::pointEigenToMsg(point_.cast<double>(), point_msg);
  tf::pointEigenToMsg((point_ + getPlaneNormal() * 1.0).cast<double>(), vec_end_msg);
  msg.points.push_back(point_msg);
  msg.points.push_back(vec_end_msg);
  msg.scale.x = 0.10;
  msg.scale.y = 0.10;
  msg.scale.z = 0.10;
  msg.color.r = 0.0;
  msg.color.g = 0.0;
  msg.color.b = 0.0;
  msg.color.a = 1.0;
  msg.id = marker_id++;
  msg.ns = "vector";
  msg.header.stamp = ros::Time::now();
  return msg;
}
// transformation accessors
/**
 * @brief Plane orientation is based on the world frame so a transformation
 * of the plane_ written in the mid pose's frame should respect that.
 *
 * @param Tnew_old
 */
void PlaneType::transformPlane(const Transformation& Tnew_old,
                               const Transformation& T_M_R) {
  Transformation T_P_P;
  transformPlane(T_P_P);
}
void PlaneType::transformPlane(const Transformation& Tnew_old) {
  T_M_P_ = T_M_P_init_ * Tnew_old;
  plane_.transform(T_M_P_.getRotationMatrix(),
                   Eigen::TransformTraits::Isometry);
  plane_.offset() = plane_.normal().dot(T_M_P_.getPosition());
  point_ = T_M_P_.getPosition();
}
float PlaneType::distSquared(const PlaneType& other) const {
  return distFunc2(plane_.normal(), point_, other.plane_.normal(),
                   other.point_);
}
float PlaneType::dist(const PlaneType& other) const {
  return sqrt(distSquared(other));
}
void PlaneType::buildPlaneOrientation() {
  Eigen::Matrix3f matRotation;
  const auto n = plane_.normal().stableNormalized();
  // const float& x = n.x();
  // const float& y = n.y();
  // const float& z = n.z();
  // float sqrt_nx_ny = sqrt(x * x + y * y);
  // if (sqrt_nx_ny > 1e-14) {
  // matRotation << y / sqrt_nx_ny, -x / sqrt_nx_ny, 0.0, x * z / sqrt_nx_ny,
  //     y * z / sqrt_nx_ny, -sqrt_nx_ny, x, y, z;
  Point p1(1.0, 0.0, 0.0);
  Point p2(0.0, 1.0, 0.0);
  Point p1_on_plane = plane_.projection(p1);
  Point p2_on_plane = plane_.projection(p2);
  if (p2_on_plane == p1_on_plane) {
    p2_on_plane = plane_.projection(Point(0.0, 0.0, 1.0));
  }
  Point plane_vec1 = (p1_on_plane - p2_on_plane).stableNormalized();
  Point plane_vec2 = n.cross(plane_vec1).stableNormalized();
  plane_vec1 = plane_vec2.cross(n).stableNormalized();
  // matRotation << plane_vec1, plane_vec2, n;
  matRotation.block<3,1>(0,0) = plane_vec1;
  matRotation.block<3,1>(0,1) = plane_vec2;
  matRotation.block<3,1>(0,2) = n;
  // if (matRotation.determinant() < 0) {
  //   matRotation.row(1)[0] *= -1;
  //   matRotation.row(1)[1] *= -1;
  //   matRotation.row(1)[2] *= -1;
  // }
  // } else {
  //   matRotation << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  // }

  T_M_P_ = Transformation(point_, Transformation::Rotation(matRotation));
}
void PlaneType::createPlaneSegmentAaBb(const std::vector<Point>& points,
                                       const double threshold_belongs) {
  num_points_ = 0;
  Point mean_point = Point(0, 0, 0);
  for (const Point& p : points) {
    if (plane_.absDistance(p) < threshold_belongs) {
      planeSegmentAaBb_.updateBoundingBoxLimits(p);
      mean_point += p;
      ++num_points_;
    }
  }
  if (num_points_ > 0) {
    point_ = plane_.projection(mean_point / num_points_);
    buildPlaneOrientation();
    T_M_P_init_ = T_M_P_;
  }
}
void PlaneType::createPlaneSegmentAaBb(const std::vector<const Point*>& points,
                                       const double threshold_belongs) {
  num_points_ = 0;
  Point mean_point = Point(0, 0, 0);
  for (const Point* p : points) {
    if (plane_.absDistance(*p) < threshold_belongs) {
      planeSegmentAaBb_.updateBoundingBoxLimits(*p);
      mean_point += *p;
      ++num_points_;
    }
  }
  if (num_points_ > 0) {
    point_ = plane_.projection(mean_point / num_points_);
    // Eigen::Vector3f n = getPlaneNormal();
    // plane_.offset() = point_.dot(n);
    buildPlaneOrientation();
    T_M_P_init_ = T_M_P_;
  }


}

void PlaneType::fixNormal(const std::vector<const Point*>& points,
                const std::vector<const Point*>& normals) {
  Eigen::Vector3f normal_vector = Eigen::Vector3f::Zero();
  for (const auto n : normals) {
    normal_vector += *n;
  }
  normal_vector /= normals.size();
  double d = 0;
  for (const auto p : points) {
    d += p->dot(normal_vector);
  }
  plane_ = Eigen::Hyperplane<float, 3>(normal_vector, static_cast<float>(d/points.size()));
  buildPlaneOrientation();
  T_M_P_init_ = T_M_P_;
}

void PlaneType::reverseNormal() {
  plane_.normal() = - plane_.normal();
  plane_.offset() = - plane_.offset();
  buildPlaneOrientation();
  T_M_P_init_ = T_M_P_;
}

void PlaneType::setplaneSegmentAaBb(const BoundingBoxType& bbox) {
  planeSegmentAaBb_.min = bbox.min;
  planeSegmentAaBb_.max = bbox.max;
}

float PlaneType::distFunc2(const Point& p1, const Point& n1, const Point& p2,
                           const Point& n2) {
  double d1 = (n2 - n1).squaredNorm();
  double d2 = (p2 - p1).dot(n2 + n1);
  return d1 + 0.5 * d2 * d2;
}
panoptic_mapping_msgs::PlaneType PlaneType::toPlaneTypeMsg() const {
  panoptic_mapping_msgs::PlaneType msg;
  msg.bbox = planeSegmentAaBb_.toBoundingBoxMsg();
  const Eigen::Vector3d point_d = point_.cast<double>();
  const Eigen::Vector3d normal_d = plane_.normal().cast<double>();
  msg.class_id = class_id_;
  tf::pointEigenToMsg(point_d, msg.point);
  tf::pointEigenToMsg(normal_d, msg.normal);
  tf::poseKindrToMsg(T_M_P_init_.cast<double>(), &msg.T_M_P);
  msg.num_points_ = num_points_;
  return msg;
}

PlaneType PlaneType::fromMsg(const panoptic_mapping_msgs::PlaneType& msg) {
  Eigen::Vector3d n_d, p_d;
  tf::pointMsgToEigen(msg.normal, n_d);
  tf::pointMsgToEigen(msg.point, p_d);
  const Point n = n_d.cast<float>();
  const Point p = p_d.cast<float>();
  const int class_id = msg.class_id;
  const size_t num_points = msg.num_points_;
  kindr::minimal::QuatTransformationTemplate<double> tf_received;
  tf::poseMsgToKindr(msg.T_M_P, &tf_received);
  const Transformation T_M_P = tf_received.cast<float>();
  PlaneType ret(n, p, T_M_P, class_id);
  ret.num_points_ = num_points;
  return ret;
}

}  // namespace voxgraph
