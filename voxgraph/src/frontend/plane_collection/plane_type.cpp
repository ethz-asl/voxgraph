#include "voxgraph/frontend/plane_collection/plane_type.h"

#include <map>
#include <vector>

#include <minkindr_conversions/kindr_msg.h>
#include <panoptic_mapping_msgs/PlaneType.h>

namespace voxgraph {

PlaneType::PlaneType(const Eigen::Vector3f& normal, const Point& point,
                     const int class_id)
    : plane_(normal, normal.dot(point)),
      point_(point),
      class_id_(class_id),
      plane_id_(getNextPlaneID()) {
  assert(normal.squaredNorm() - 1 < 1.000001 &&
         1 - normal.squaredNorm() > 0.9999);
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
  point_ = plane.offset() * n;
  T_M_P_ = T_M_P;
  T_M_P_init_ = T_M_P;
  num_points_ = 0;
}
PlaneType::PlaneType(const Eigen::Vector3f& normal, const Point& point,
                     const Transformation& T_M_P, int class_id)
    : PlaneType(Eigen::Hyperplane<float, 3>(normal, point), T_M_P, class_id) {
  assert(normal.squaredNorm() - 1 < 1.000001 &&
         1 - normal.squaredNorm() > 0.9999);
}
// getters
Eigen::Vector3f PlaneType::getPlaneNormal() const {
  return plane_.normal().normalized();
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
  const auto n = plane_.normal().normalized();
  const float& x = n.x();
  const float& y = n.y();
  const float& z = n.z();
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
  Point plane_vec1 = (p1_on_plane - p2_on_plane).normalized();
  Point plane_vec2 = n.cross(plane_vec1).normalized();
  plane_vec1 = plane_vec2.cross(n).normalized();
  matRotation << plane_vec1, plane_vec2, n;
  if (matRotation.determinant() < 0) {
    matRotation.row(1)[0] *= -1;
    matRotation.row(1)[1] *= -1;
    matRotation.row(1)[2] *= -1;
  }
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
  }
  LOG(INFO) << "plane segment aabb created with " << num_points_
            << " points inside";
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
  const PlaneType ret(n, p, T_M_P, class_id);
  return ret;
}

}  // namespace voxgraph
