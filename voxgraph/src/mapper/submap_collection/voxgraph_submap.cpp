//
// Created by victor on 17.01.19.
//

#include "voxgraph/mapper/submap_collection/voxgraph_submap.h"

namespace voxgraph {
const VoxgraphSubmap::BoundingBox VoxgraphSubmap::getSubmapFrameSurfaceObb()
    const {
  // Check if the OBB has yet been measured
  if ((surface_obb_.min.array() > surface_obb_.max.array()).any()) {
    // After any update, the min point coefficients should be smaller or equal
    // to their max point counterparts. Because this was not the case, we assume
    // that they have not yet been updated since their +/- Inf initialization.
    // Get list of all allocated voxel blocks
    voxblox::BlockIndexList block_list;
    tsdf_map_->getTsdfLayer().getAllAllocatedBlocks(&block_list);

    // Calculate the number of voxels per block
    size_t vps = tsdf_map_->getTsdfLayer().voxels_per_side();
    size_t num_voxels_per_block = vps * vps * vps;

    // Create a vector spanning from a voxel's center to its max corner
    const voxblox::Point half_voxel_size =
        0.5 * tsdf_map_->getTsdfLayer().voxel_size() * voxblox::Point::Ones();

    // Iterate over all allocated blocks in the submap
    for (const voxblox::BlockIndex &block_index : block_list) {
      const voxblox::Block<voxblox::TsdfVoxel> &block =
          tsdf_map_->getTsdfLayer().getBlockByIndex(block_index);
      // Iterate over all voxels in block
      for (size_t linear_index = 0u; linear_index < num_voxels_per_block;
           ++linear_index) {
        const voxblox::TsdfVoxel &voxel =
            block.getVoxelByLinearIndex(linear_index);
        // Select the observed voxels within the truncation band
        // TODO(victorr): Set these constants from params
        if (voxel.weight > 1e-6 && std::abs(voxel.distance) < 0.3) {
          // Update the min and max points of the OBB
          voxblox::Point voxel_coordinates =
              block.computeCoordinatesFromLinearIndex(linear_index);
          surface_obb_.min =
              surface_obb_.min.cwiseMin(voxel_coordinates - half_voxel_size);
          surface_obb_.max =
              surface_obb_.max.cwiseMax(voxel_coordinates + half_voxel_size);
        }
      }
    }
  }
  return surface_obb_;
}

const VoxgraphSubmap::BoundingBox VoxgraphSubmap::getSubmapFrameSubmapObb()
    const {
  // Check if the OBB has yet been measured
  if ((map_obb_.min.array() > map_obb_.max.array()).any()) {
    // After any update, the min point coefficients should be smaller or equal
    // to their max point counterparts. Because this was not the case, we assume
    // that they have not yet been updated since their +/- Inf initialization.
    // Get list of all allocated voxel blocks
    voxblox::BlockIndexList block_list;
    tsdf_map_->getTsdfLayer().getAllAllocatedBlocks(&block_list);

    // Create a vector spanning from a block's center to its max corner
    const voxblox::Point half_block_size =
        0.5 * block_size() * voxblox::Point::Ones();

    // Iterate over all allocated blocks in the submap
    for (const voxblox::BlockIndex &block_index : block_list) {
      // Update the min and max points of the OBB
      voxblox::Point block_coordinates =
          voxblox::getCenterPointFromGridIndex(block_index, block_size());
      map_obb_.min = map_obb_.min.cwiseMin(block_coordinates - half_block_size);
      map_obb_.max = map_obb_.max.cwiseMax(block_coordinates + half_block_size);
    }
  }
  return map_obb_;
}

const VoxgraphSubmap::BoxCornerMatrix
VoxgraphSubmap::getWorldFrameSurfaceObbCorners() const {
  // Create a matrix whose columns are the box corners' homogeneous coordinates
  HomogBoxCornerMatrix box_corner_matrix = HomogBoxCornerMatrix::Constant(1);
  box_corner_matrix.topLeftCorner(3, 8) =
      getSubmapFrameSurfaceObb().getCornerCoordinates();
  // Transform the box corner coordinates to world frame
  box_corner_matrix = getPose().getTransformationMatrix() * box_corner_matrix;
  return box_corner_matrix.topLeftCorner(3, 8);
}

const VoxgraphSubmap::BoxCornerMatrix
VoxgraphSubmap::getWorldFrameSubmapObbCorners() const {
  // Create a matrix whose columns are the box corners' homogeneous coordinates
  HomogBoxCornerMatrix box_corner_matrix = HomogBoxCornerMatrix::Constant(1);
  box_corner_matrix.topLeftCorner(3, 8) =
      getSubmapFrameSubmapObb().getCornerCoordinates();
  // Transform the box corner coordinates to world frame
  box_corner_matrix = getPose().getTransformationMatrix() * box_corner_matrix;
  return box_corner_matrix.topLeftCorner(3, 8);
}

const VoxgraphSubmap::BoundingBox VoxgraphSubmap::getWorldFrameSurfaceAabb()
    const {
  // Return the Axis Aligned Bounding Box around the isosurface in world frame
  return BoundingBox::getAabbFromObbAndPose(getSubmapFrameSurfaceObb(),
                                            getPose());
}

const VoxgraphSubmap::BoundingBox VoxgraphSubmap::getWorldFrameSubmapAabb()
    const {
  // Return the Axis Aligned Bounding Box around the full submap in world frame
  return BoundingBox::getAabbFromObbAndPose(getSubmapFrameSubmapObb(),
                                            getPose());
}

const VoxgraphSubmap::BoxCornerMatrix
VoxgraphSubmap::getWorldFrameSurfaceAabbCorners() const {
  // Return a matrix whose columns are the corner points
  // of the submap's surface AABB
  return getWorldFrameSurfaceAabb().getCornerCoordinates();
}

const VoxgraphSubmap::BoxCornerMatrix
VoxgraphSubmap::getWorldFrameSubmapAabbCorners() const {
  // Return a matrix whose columns are the corner points
  // of the full submap's AABB
  return getWorldFrameSubmapAabb().getCornerCoordinates();
}

bool VoxgraphSubmap::overlapsWith(const VoxgraphSubmap &otherSubmap) const {
  // TODO(victorr): Implement improved overlap test
  const BoundingBox aabb = getWorldFrameSurfaceAabb();
  const BoundingBox other_aabb = otherSubmap.getWorldFrameSubmapAabb();
  // If there's a separation along any of the 3 axes, the AABBs don't intersect
  if (aabb.max[0] < other_aabb.min[0] || aabb.min[0] > other_aabb.max[0])
    return false;
  if (aabb.max[1] < other_aabb.min[1] || aabb.min[1] > other_aabb.max[1])
    return false;
  if (aabb.max[2] < other_aabb.min[2] || aabb.min[2] > other_aabb.max[2])
    return false;
  // The AABBs overlap on all axes: therefore they must be intersecting
  return true;
}

}  // namespace voxgraph
