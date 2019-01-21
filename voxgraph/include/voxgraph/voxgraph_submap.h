//
// Created by victor on 08.01.19.
//

#ifndef VOXGRAPH_VOXGRAPH_SUBMAP_H_
#define VOXGRAPH_VOXGRAPH_SUBMAP_H_

#include <cblox/core/tsdf_esdf_submap.h>
#include <bitset>
#include <memory>

namespace voxgraph {
class VoxgraphSubmap : public cblox::TsdfEsdfSubmap {
 public:
  typedef std::shared_ptr<VoxgraphSubmap> Ptr;
  typedef std::shared_ptr<const VoxgraphSubmap> ConstPtr;

  typedef Eigen::Matrix<voxblox::FloatingPoint, 3, 8> BoxCornerMatrix;
  struct BoundingBox {
    voxblox::Point min = {INFINITY, INFINITY, INFINITY};
    voxblox::Point max = {-INFINITY, -INFINITY, -INFINITY};
    const BoxCornerMatrix getCornerCoordinates() const {
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
    static const BoundingBox getAabbFromObbAndPose(
        const BoundingBox &obb, const voxblox::Transformation &pose) {
      // Create AABB
      BoundingBox aabb;
      // Transform the OBB corners into world frame and update the AABB
      const BoxCornerMatrix submap_t_submap__obb_corners =
          obb.getCornerCoordinates();
      for (int i = 0; i < 8; i++) {
        const voxblox::Point world_t_world__obb_corner =
            pose * submap_t_submap__obb_corners.col(i);
        aabb.min = aabb.min.cwiseMin(world_t_world__obb_corner);
        aabb.max = aabb.max.cwiseMax(world_t_world__obb_corner);
      }
      return aabb;
    }
  };

  VoxgraphSubmap(const voxblox::Transformation &T_M_S,
                 cblox::SubmapID submap_id, Config config)
      : cblox::TsdfEsdfSubmap(T_M_S, submap_id, config) {}

  const BoundingBox getSubmapFrameSurfaceObb() const;
  const BoundingBox getSubmapFrameSubmapObb() const;
  const BoundingBox getWorldFrameSurfaceAabb() const;
  const BoundingBox getWorldFrameSubmapAabb() const;
  const BoxCornerMatrix getWorldFrameSurfaceObbCorners() const;
  const BoxCornerMatrix getWorldFrameSubmapObbCorners() const;
  const BoxCornerMatrix getWorldFrameSurfaceAabbCorners() const;
  const BoxCornerMatrix getWorldFrameSubmapAabbCorners() const;

  bool overlapsWith(VoxgraphSubmap::ConstPtr otherSubmap) const;

  // TODO(victorr): Move RelevantVoxelList from registration_cost_function here

 private:
  // Oriented Bounding Boxes in submap frame
  mutable BoundingBox surface_obb_;  // around the isosurface
  mutable BoundingBox map_obb_;      // around the entire submap

  typedef Eigen::Matrix<voxblox::FloatingPoint, 4, 8> HomogBoxCornerMatrix;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_VOXGRAPH_SUBMAP_H_
