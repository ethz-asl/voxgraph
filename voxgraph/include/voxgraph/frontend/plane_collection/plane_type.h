#ifndef VOXGRAPH_FRONTEND_PLANE_COLLECTION_PLANE_TYPE_H_
#define VOXGRAPH_FRONTEND_PLANE_COLLECTION_PLANE_TYPE_H_

#include <map>
#include <vector>

#include <minkindr_conversions/kindr_msg.h>
#include <panoptic_mapping_msgs/PlaneType.h>
#include <voxblox/core/common.h>

#include "voxgraph/common.h"
#include "voxgraph/frontend/plane_collection/bounding_box_extended.h"

namespace voxgraph {

using BoundingBoxType = BoundingBoxExtended;
using Point = voxblox::Point;

class PlaneType {
 public:
  typedef const PlaneType* ConstPtr;
  typedef size_t PlaneID;
  explicit PlaneType(const Eigen::Vector3f& normal, const Point& point,
                     const int class_id);
  explicit PlaneType(const Eigen::Hyperplane<float, 3>& plane, int class_id);
  explicit PlaneType(const Eigen::Hyperplane<float, 3>& plane,
                     const Transformation& T_M_P, int class_id);
  explicit PlaneType(const Eigen::Vector3f& normal, const Point& point,
                     const Transformation& T_M_P, int class_id);
  // getters
  Eigen::Vector3f getPlaneNormal() const;
  Point getPointInit() const;
  Transformation getPlaneTransformation() const;
  PlaneID getPlaneID() const;
  const voxgraph::BoundingBox* getBoundingBox() const;
  visualization_msgs::Marker getVisualizationMsg() const;
  // transformation accessors
  /**
   * @brief Plane orientation is based on the world frame so a transformation
   * of the plane_ written in the mid pose's frame should respect that.
   *
   * @param Tnew_old
   */
  void transformPlane(const Transformation& Tnew_old,
                      const Transformation& T_M_R);
  void transformPlane(const Transformation& Tnew_old);
  float distSquared(const PlaneType& other) const;
  float dist(const PlaneType& other) const;
  void buildPlaneOrientation();
  void createPlaneSegmentAaBb(const std::vector<Point>& points,
                              const double threshold_belongs);
  void createPlaneSegmentAaBb(const std::vector<const Point*>& points,
                              const double threshold_belongs);
  void fixNormal(const std::vector<const Point*>& points,
                 const std::vector<const Point*>& normals);
  void reverseNormal();
  void setplaneSegmentAaBb(const BoundingBoxType& bbox);

  static float distFunc2(const Point& p1, const Point& n1, const Point& p2,
                         const Point& n2);
  panoptic_mapping_msgs::PlaneType toPlaneTypeMsg() const;
  static PlaneType fromMsg(const panoptic_mapping_msgs::PlaneType& msg);
  size_t getNumPoints() const { return num_points_; }
 private:
  PlaneID plane_id_;
  static PlaneID getNextPlaneID() {
    static int next_plane_id = 0;
    return next_plane_id++;
  }
  int class_id_ = -2;
  Transformation T_M_P_;
  Transformation T_M_P_init_;
  Point point_;
  Eigen::Hyperplane<float, 3> plane_;
  size_t num_points_ = 0u;
  BoundingBoxType planeSegmentAaBb_;
};

typedef std::map<int, std::vector<PlaneType> > classToPlanesType;

}  // namespace voxgraph

#endif  // VOXGRAPH_FRONTEND_PLANE_COLLECTION_PLANE_TYPE_H_
