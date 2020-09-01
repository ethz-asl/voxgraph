#ifndef VOXGRAPH_FRONTEND_SUBMAP_COLLECTION_BOUNDING_BOX_H_
#define VOXGRAPH_FRONTEND_SUBMAP_COLLECTION_BOUNDING_BOX_H_

#include <voxblox/core/common.h>
#include <Eigen/Dense>

namespace voxgraph {
typedef Eigen::Matrix<voxblox::FloatingPoint, 3, 8> BoxCornerMatrix;
class BoundingBox {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  voxblox::Point min = {INFINITY, INFINITY, INFINITY};
  voxblox::Point max = {-INFINITY, -INFINITY, -INFINITY};

  void reset();

  const BoxCornerMatrix getCornerCoordinates() const;

  static const BoundingBox getAabbFromObbAndPose(
      const BoundingBox& obb, const voxblox::Transformation& pose);

  bool overlapsWith(const BoundingBox& other_bounding_box) const {
    // If there's a separation along any of the 3 axes, the AABBs don't
    // intersect
    if (max[0] < other_bounding_box.min[0] ||
        min[0] > other_bounding_box.max[0])
      return false;
    if (max[1] < other_bounding_box.min[1] ||
        min[1] > other_bounding_box.max[1])
      return false;
    if (max[2] < other_bounding_box.min[2] ||
        min[2] > other_bounding_box.max[2])
      return false;
    // Since the AABBs overlap on all axes, the submaps could be overlapping
    return true;
  }
};
}  // namespace voxgraph

#endif  // VOXGRAPH_FRONTEND_SUBMAP_COLLECTION_BOUNDING_BOX_H_
