//
// Created by victor on 17.01.19.
//

#include "voxgraph/voxgraph_submap.h"

namespace voxgraph {
const VoxgraphSubmap::BoundingBox VoxgraphSubmap::getSurfaceObb() const {
  // Check if the OBB has yet been measured
  if (surface_obb_.min == surface_obb_.max) {
    // Since its extremities are equal, we assume they still has to be found
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

const VoxgraphSubmap::BoundingBox VoxgraphSubmap::getMapObb() const {
  // Check if the OBB has yet been measured
  if (map_obb_.min == map_obb_.max) {
    // Since its extremities are equal, we assume they still has to be found
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

}  // namespace voxgraph
