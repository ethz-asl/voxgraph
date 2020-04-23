#include "voxgraph/frontend/submap_collection/bounding_box.h"

#include <bitset>
#include <memory>

namespace voxgraph {
void BoundingBox::reset() {
  min = {INFINITY, INFINITY, INFINITY};
  max = {-INFINITY, -INFINITY, -INFINITY};
}

const BoxCornerMatrix BoundingBox::getCornerCoordinates() const {
  BoxCornerMatrix box_corners;
  // The set of box corners is composed by taking all combinations of
  // coefficients from the min and max point, i.e. 2^3=8 combinations.
  // Since these combinations correspond to the bit table of an int
  // going from 0 to 7, we can use:
  std::bitset<3> bit_table_row;
  for (unsigned int i = 0; i < 8; i++) {
    bit_table_row = std::bitset<3>(i);
    box_corners(0, i) = bit_table_row[0] ? min.x() : max.x();
    box_corners(1, i) = bit_table_row[1] ? min.y() : max.y();
    box_corners(2, i) = bit_table_row[2] ? min.z() : max.z();
  }
  return box_corners;
}

const BoundingBox BoundingBox::getAabbFromObbAndPose(
    const BoundingBox& obb, const voxblox::Transformation& pose) {
  // Create AABB
  BoundingBox aabb;
  // Transform the OBB corners into mission frame and update the AABB
  const BoxCornerMatrix submap_t_submap__obb_corners =
      obb.getCornerCoordinates();
  for (int i = 0; i < 8; i++) {
    const voxblox::Point mission_t_mission__obb_corner =
        pose * submap_t_submap__obb_corners.col(i);
    aabb.min = aabb.min.cwiseMin(mission_t_mission__obb_corner);
    aabb.max = aabb.max.cwiseMax(mission_t_mission__obb_corner);
  }
  return aabb;
}
}  // namespace voxgraph
