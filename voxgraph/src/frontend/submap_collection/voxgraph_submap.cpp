//
// Created by victor on 17.01.19.
//

#include "voxgraph/frontend/submap_collection/voxgraph_submap.h"
#include <voxblox/integrator/merge_integration.h>
#include <voxblox/interpolator/interpolator.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <utility>

namespace voxgraph {
VoxgraphSubmap::VoxgraphSubmap(const voxblox::Transformation &T_M_S,
                               const cblox::SubmapID &submap_id,
                               const voxgraph::VoxgraphSubmap::Config &config)
    : cblox::TsdfEsdfSubmap(T_M_S, submap_id, config), config_(config) {}

VoxgraphSubmap::VoxgraphSubmap(
    const voxblox::Transformation &T_M_S, const cblox::SubmapID &submap_id,
    const voxblox::Layer<voxblox::TsdfVoxel> &tsdf_layer)
    : cblox::TsdfEsdfSubmap(T_M_S, submap_id, Config()) {
  // Update the inherited TsdfEsdfSubmap config
  config_.tsdf_voxel_size = tsdf_layer.voxel_size();
  config_.tsdf_voxels_per_side = tsdf_layer.voxels_per_side();
  config_.esdf_voxel_size = tsdf_layer.voxel_size();
  config_.esdf_voxels_per_side = tsdf_layer.voxels_per_side();

  // Reset the inherited EsdfMap
  esdf_map_.reset(new voxblox::EsdfMap(config_));

  // Reset the inherited TsdfMap to contain a copy of the provided tsdf_layer
  tsdf_map_.reset(new voxblox::TsdfMap(tsdf_layer));
}

void VoxgraphSubmap::transformSubmap(const voxblox::Transformation &T_new_old) {
  // Transform TSDF
  voxblox::Layer<voxblox::TsdfVoxel> old_tsdf_layer(tsdf_map_->getTsdfLayer());
  voxblox::transformLayer(old_tsdf_layer, T_new_old,
                          tsdf_map_->getTsdfLayerPtr());
  // Reset cached Oriented Bounding Boxes
  surface_obb_.reset();
  map_obb_.reset();

  // Transform pose history
  for (std::pair<const ros::Time, voxblox::Transformation> &kv :
       pose_history_) {
    kv.second = T_new_old * kv.second;
  }

  // Transform the submap pose
  setPose(getPose() * T_new_old.inverse());

  // Regenerate all cached values
  finishSubmap();
}

void VoxgraphSubmap::addPoseToHistory(
    const ros::Time &timestamp, const voxblox::Transformation &T_world_robot) {
  voxblox::Transformation T_submap_robot = getPose().inverse() * T_world_robot;
  pose_history_.emplace(timestamp, T_submap_robot);
}

void VoxgraphSubmap::finishSubmap() {
  // Generate the cached the ESDF
  generateEsdf();

  // Generate the cached Oriented Bounded Boxes
  getSubmapFrameSubmapObb();
  getSubmapFrameSurfaceObb();

  // Populate the relevant block voxel index hash map
  findRelevantVoxelIndices();
  std::cout << "# relevant voxels: " << num_relevant_voxels_ << std::endl;

  // Populate the isosurface vertex vector
  findIsosurfaceVertices();
  std::cout << "# isosurface vertices: " << num_isosurface_vertices_
            << std::endl;

  // Set the finished flag
  finished_ = true;
}

const ros::Time VoxgraphSubmap::getCreationTime() const {
  if (pose_history_.empty()) {
    return ros::Time(0);
  } else {
    return (pose_history_.begin())->first;
  }
}

const voxblox::HierarchicalIndexMap &
VoxgraphSubmap::getRelevantBlockVoxelIndices() const {
  CHECK(finished_) << "The cached relevant voxel index list is only available"
                      " once the submap has been declared finished.";
  return relevant_block_voxel_indices_;
}

const unsigned int VoxgraphSubmap::getNumRelevantVoxels() const {
  return num_relevant_voxels_;
}

void VoxgraphSubmap::findRelevantVoxelIndices() {
  // Get a reference to the TSDF layer
  const voxblox::Layer<voxblox::TsdfVoxel> &tsdf_layer =
      tsdf_map_->getTsdfLayer();

  // Get list of all allocated voxel blocks in the reference submap
  voxblox::BlockIndexList block_list;
  tsdf_layer.getAllAllocatedBlocks(&block_list);

  // Calculate the number of voxels per block
  size_t vps = tsdf_map_->getTsdfLayer().voxels_per_side();
  size_t num_voxels_per_block = vps * vps * vps;

  // Create a list containing only reference voxels that matter
  // i.e. that have been observed and fall within a truncation band
  // Iterate over all allocated blocks in the reference submap
  for (const voxblox::BlockIndex &block_index : block_list) {
    const voxblox::Block<voxblox::TsdfVoxel> &reference_block =
        tsdf_layer.getBlockByIndex(block_index);
    // Iterate over all voxels in block
    for (size_t linear_index = 0u; linear_index < num_voxels_per_block;
         ++linear_index) {
      const voxblox::TsdfVoxel &ref_voxel =
          reference_block.getVoxelByLinearIndex(linear_index);
      // Select the observed voxels within the truncation band
      if (ref_voxel.weight > config_.min_voxel_weight &&
          std::abs(ref_voxel.distance) < config_.max_voxel_distance) {
        voxblox::VoxelIndex voxel_index =
            reference_block.computeVoxelIndexFromLinearIndex(linear_index);
        relevant_block_voxel_indices_[block_index].push_back(voxel_index);
        num_relevant_voxels_++;
      }
    }
  }
}

const voxblox::Pointcloud &VoxgraphSubmap::getIsosurfaceVertices() const {
  CHECK(finished_) << "The cached isosurface vertex vector is only available"
                      " once the submap has been declared finished.";
  return isosurface_vertices_;
}

const unsigned int VoxgraphSubmap::getNumIsosurfaceVertices() const {
  return num_isosurface_vertices_;
}

void VoxgraphSubmap::findIsosurfaceVertices() {
  // Generate the mesh layer
  voxblox::MeshLayer mesh_layer(tsdf_map_->block_size());
  voxblox::MeshIntegratorConfig mesh_integrator_config;
  mesh_integrator_config.use_color = false;
  mesh_integrator_config.min_weight = config_.min_voxel_weight;
  voxblox::MeshIntegrator<voxblox::TsdfVoxel> mesh_integrator(
      mesh_integrator_config, tsdf_map_->getTsdfLayer(), &mesh_layer);
  mesh_integrator.generateMesh(false, false);

  // Convert it into a connected mesh
  voxblox::Point origin{0, 0, 0};
  voxblox::Mesh connected_mesh_layer(tsdf_map_->block_size(), origin);
  mesh_layer.getConnectedMesh(&connected_mesh_layer,
                              0.5 * tsdf_map_->voxel_size());

  // Extract the vertices
  isosurface_vertices_ = connected_mesh_layer.vertices;
  num_isosurface_vertices_ = isosurface_vertices_.size();
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

const BoundingBox VoxgraphSubmap::getSubmapFrameSurfaceObb() const {
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
        if (voxel.weight > config_.min_voxel_weight &&
            std::abs(voxel.distance) < config_.max_voxel_distance) {
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

const BoundingBox VoxgraphSubmap::getSubmapFrameSubmapObb() const {
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

const BoxCornerMatrix VoxgraphSubmap::getWorldFrameSurfaceObbCorners() const {
  // Create a matrix whose columns are the box corners' homogeneous coordinates
  HomogBoxCornerMatrix box_corner_matrix = HomogBoxCornerMatrix::Constant(1);
  box_corner_matrix.topLeftCorner(3, 8) =
      getSubmapFrameSurfaceObb().getCornerCoordinates();
  // Transform the box corner coordinates to world frame
  box_corner_matrix = getPose().getTransformationMatrix() * box_corner_matrix;
  return box_corner_matrix.topLeftCorner(3, 8);
}

const BoxCornerMatrix VoxgraphSubmap::getWorldFrameSubmapObbCorners() const {
  // Create a matrix whose columns are the box corners' homogeneous coordinates
  HomogBoxCornerMatrix box_corner_matrix = HomogBoxCornerMatrix::Constant(1);
  box_corner_matrix.topLeftCorner(3, 8) =
      getSubmapFrameSubmapObb().getCornerCoordinates();
  // Transform the box corner coordinates to world frame
  box_corner_matrix = getPose().getTransformationMatrix() * box_corner_matrix;
  return box_corner_matrix.topLeftCorner(3, 8);
}

const BoundingBox VoxgraphSubmap::getWorldFrameSurfaceAabb() const {
  // Return the Axis Aligned Bounding Box around the isosurface in world frame
  return BoundingBox::getAabbFromObbAndPose(getSubmapFrameSurfaceObb(),
                                            getPose());
}

const BoundingBox VoxgraphSubmap::getWorldFrameSubmapAabb() const {
  // Return the Axis Aligned Bounding Box around the full submap in world frame
  return BoundingBox::getAabbFromObbAndPose(getSubmapFrameSubmapObb(),
                                            getPose());
}

const BoxCornerMatrix VoxgraphSubmap::getWorldFrameSurfaceAabbCorners() const {
  // Return a matrix whose columns are the corner points
  // of the submap's surface AABB
  return getWorldFrameSurfaceAabb().getCornerCoordinates();
}

const BoxCornerMatrix VoxgraphSubmap::getWorldFrameSubmapAabbCorners() const {
  // Return a matrix whose columns are the corner points
  // of the full submap's AABB
  return getWorldFrameSubmapAabb().getCornerCoordinates();
}
}  // namespace voxgraph
