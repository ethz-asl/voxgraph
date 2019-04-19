//
// Created by victor on 06.04.19.
//

#ifndef VOXGRAPH_FRONTEND_SUBMAP_COLLECTION_BOUNDING_BOX_H_
#define VOXGRAPH_FRONTEND_SUBMAP_COLLECTION_BOUNDING_BOX_H_

#include <voxblox/core/common.h>
#include <Eigen/Dense>

namespace voxgraph {
typedef Eigen::Matrix<voxblox::FloatingPoint, 3, 8> BoxCornerMatrix;
class BoundingBox {
 public:
  voxblox::Point min = {INFINITY, INFINITY, INFINITY};
  voxblox::Point max = {-INFINITY, -INFINITY, -INFINITY};

  void reset();

  const BoxCornerMatrix getCornerCoordinates() const;

  static const BoundingBox getAabbFromObbAndPose(
      const BoundingBox &obb, const voxblox::Transformation &pose);
};
}  // namespace voxgraph

#endif  // VOXGRAPH_FRONTEND_SUBMAP_COLLECTION_BOUNDING_BOX_H_
