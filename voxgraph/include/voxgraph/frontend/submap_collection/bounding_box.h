#ifndef VOXGRAPH_FRONTEND_SUBMAP_COLLECTION_BOUNDING_BOX_H_
#define VOXGRAPH_FRONTEND_SUBMAP_COLLECTION_BOUNDING_BOX_H_

#include <Eigen/Dense>
#include <voxblox/core/common.h>

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
};
}  // namespace voxgraph

#endif  // VOXGRAPH_FRONTEND_SUBMAP_COLLECTION_BOUNDING_BOX_H_
