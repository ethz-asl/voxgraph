#include "voxgraph/frontend/submap_collection/voxgraph_submap.h"

#include <memory>
#include <utility>

#include <voxblox/integrator/merge_integration.h>
#include <voxblox/interpolator/interpolator.h>
#include <voxblox/mesh/mesh_integrator.h>

namespace voxgraph {
VoxgraphSubmap::VoxgraphSubmap(const voxblox::Transformation& T_O_S,
                               const cblox::SubmapID& submap_id,
                               const voxgraph::VoxgraphSubmap::Config& config)
    : cblox::TsdfEsdfSubmap(T_O_S, submap_id, config), config_(config) {}

VoxgraphSubmap::VoxgraphSubmap(
    const voxblox::Transformation& T_O_S, const cblox::SubmapID& submap_id,
    const voxblox::Layer<voxblox::TsdfVoxel>& tsdf_layer)
    : cblox::TsdfEsdfSubmap(T_O_S, submap_id, Config()) {
  // Update the inherited TsdfEsdfSubmap config
  config_.tsdf_voxel_size = tsdf_layer.voxel_size();
  config_.tsdf_voxels_per_side = tsdf_layer.voxels_per_side();
  config_.esdf_voxel_size = tsdf_layer.voxel_size();
  config_.esdf_voxels_per_side = tsdf_layer.voxels_per_side();

  // Reset the inherited EsdfMap
  esdf_map_ = std::make_shared<voxblox::EsdfMap>(config_);

  // Reset the inherited TsdfMap to contain a copy of the provided tsdf_layer
  tsdf_map_ = std::make_shared<voxblox::TsdfMap>(tsdf_layer);
}

void VoxgraphSubmap::transformSubmap(const voxblox::Transformation& T_new_old) {
  // Transform TSDF
  voxblox::Layer<voxblox::TsdfVoxel> old_tsdf_layer(tsdf_map_->getTsdfLayer());
  tsdf_map_->getTsdfLayerPtr()->removeAllBlocks();
  voxblox::transformLayer(old_tsdf_layer, T_new_old,
                          tsdf_map_->getTsdfLayerPtr());
  // Reset cached Oriented Bounding Boxes
  surface_obb_.reset();
  map_obb_.reset();

  // Transform pose history
  for (std::pair<const ros::Time, voxblox::Transformation>& kv :
       pose_history_) {
    kv.second = T_new_old * kv.second;
  }

  // Transform the submap pose
  setPose(getPose() * T_new_old.inverse());

  // Regenerate all cached values
  finishSubmap();
}

void VoxgraphSubmap::addPoseToHistory(
    const ros::Time& timestamp, const voxblox::Transformation& T_submap_base) {
  pose_history_.emplace(timestamp, T_submap_base);
}

bool VoxgraphSubmap::lookupPoseByTime(
    const ros::Time& timestamp, voxblox::Transformation* T_submap_robot) const {
  CHECK_NOTNULL(T_submap_robot);
  // TODO(victorr): Check if timestamp falls between submap start and end
  //                timestamps and return false otherwise

  // Get an iterator to the end of the time interval in which timestamp falls
  auto iterator = pose_history_.upper_bound(timestamp);

  // Ensure that the timestamp is not from before this submap was first used
  if (iterator == pose_history_.begin()) {
    return false;
  }

  // TODO(victorr): Use linear interpolation instead of 0th order hold
  // The interval's starting transform id is stored at its start point
  iterator--;
  *T_submap_robot = iterator->second;
  return true;
}

void VoxgraphSubmap::finishSubmap() {
  // Generate the cached the ESDF
  generateEsdf();

  // Generate the cached Oriented Bounded Boxes
  getSubmapFrameSubmapObb();
  getSubmapFrameSurfaceObb();

  // Populate the relevant block voxel index hash map
  findRelevantVoxelIndices();
  std::cout << "\n# relevant voxels: " << relevant_voxels_.size() << std::endl;

  // Populate the isosurface vertex vector
  findIsosurfaceVertices();
  std::cout << "\n# isosurface vertices: " << isosurface_vertices_.size()
            << std::endl;

  // Set mapping times
  mapping_interval_.first = getStartTime().toNSec();
  mapping_interval_.second = getEndTime().toNSec();

  // Set the finished flag
  finished_ = true;
}

void VoxgraphSubmap::setRegistrationFilterConfig(
    const Config::RegistrationFilter& registration_filter_config) {
  // Update registration filter config
  config_.registration_filter = registration_filter_config;
}

const ros::Time VoxgraphSubmap::getStartTime() const {
  if (pose_history_.empty()) {
    return ros::Time(0);
  } else {
    return (pose_history_.begin())->first;
  }
}

const ros::Time VoxgraphSubmap::getEndTime() const {
  if (pose_history_.empty()) {
    return ros::Time(0);
  } else {
    return (--pose_history_.end())->first;
  }
}

const WeightedSampler<RegistrationPoint>& VoxgraphSubmap::getRegistrationPoints(
    RegistrationPointType registration_point_type) const {
  CHECK(finished_) << "The cached registration points are only available"
                      " once the submap has been declared finished.";
  switch (registration_point_type) {
    case RegistrationPointType::kVoxels:
      return relevant_voxels_;
    case RegistrationPointType::kIsosurfacePoints:
    default:
      return isosurface_vertices_;
  }
}

void VoxgraphSubmap::findRelevantVoxelIndices() {
  // Reset the cached relevant voxels
  relevant_voxels_.clear();

  // Get references to the TSDF and ESDF layers
  const TsdfLayer& tsdf_layer = tsdf_map_->getTsdfLayer();
  const EsdfLayer& esdf_layer = esdf_map_->getEsdfLayer();

  // Get list of all allocated voxel blocks in the submap
  voxblox::BlockIndexList block_list;
  tsdf_layer.getAllAllocatedBlocks(&block_list);

  // Calculate the number of voxels per block
  size_t vps = tsdf_map_->getTsdfLayer().voxels_per_side();
  size_t num_voxels_per_block = vps * vps * vps;

  // Iterate over all allocated blocks in the submap
  double min_voxel_weight = config_.registration_filter.min_voxel_weight;
  double max_voxel_distance = config_.registration_filter.max_voxel_distance;
  for (const voxblox::BlockIndex& block_index : block_list) {
    // Get the TSDF block and ESDF block if needed
    const TsdfBlock& tsdf_block = tsdf_layer.getBlockByIndex(block_index);
    EsdfBlock::ConstPtr esdf_block;
    if (config_.registration_filter.use_esdf_distance) {
      esdf_block = esdf_layer.getBlockPtrByIndex(block_index);
    }

    // Iterate over all voxels in block
    for (size_t linear_index = 0u; linear_index < num_voxels_per_block;
         ++linear_index) {
      const TsdfVoxel& tsdf_voxel =
          tsdf_block.getVoxelByLinearIndex(linear_index);
      // Select the observed voxels within the truncation band
      if (tsdf_voxel.weight > min_voxel_weight &&
          std::abs(tsdf_voxel.distance) < max_voxel_distance) {
        // Get the voxel position
        voxblox::Point voxel_coordinates =
            tsdf_block.computeCoordinatesFromLinearIndex(linear_index);

        // Get the distance at the voxel
        float distance;
        if (config_.registration_filter.use_esdf_distance) {
          CHECK(esdf_block->isValidLinearIndex(linear_index));
          const EsdfVoxel& esdf_voxel =
              esdf_block->getVoxelByLinearIndex(linear_index);
          distance = esdf_voxel.distance;
        } else {
          distance = tsdf_voxel.distance;
        }

        // Store the relevant voxel
        RegistrationPoint relevant_voxel{voxel_coordinates, distance,
                                         tsdf_voxel.weight};
        relevant_voxels_.addItem(relevant_voxel, tsdf_voxel.weight);
      }
    }
  }
}

void VoxgraphSubmap::findIsosurfaceVertices() {
  // Reset the cached isosurface vertex sample container
  isosurface_vertices_.clear();

  // Generate the mesh layer
  voxblox::MeshLayer mesh_layer(tsdf_map_->block_size());
  voxblox::MeshIntegratorConfig mesh_integrator_config;
  mesh_integrator_config.use_color = false;
  mesh_integrator_config.min_weight =
      static_cast<float>(config_.registration_filter.min_voxel_weight);
  voxblox::MeshIntegrator<voxblox::TsdfVoxel> mesh_integrator(
      mesh_integrator_config, tsdf_map_->getTsdfLayer(), &mesh_layer);
  mesh_integrator.generateMesh(false, false);

  // Convert it into a connected mesh
  voxblox::Point origin{0, 0, 0};
  voxblox::Mesh connected_mesh(tsdf_map_->block_size(), origin);
  mesh_layer.getConnectedMesh(&connected_mesh, 0.5 * tsdf_map_->voxel_size());

  // Create an interpolator to interpolate the vertex weights from the TSDF
  Interpolator tsdf_interpolator(tsdf_map_->getTsdfLayerPtr());

  // Extract the vertices
  for (const auto& mesh_vertex_coordinates : connected_mesh.vertices) {
    // Try to interpolate the voxel weight
    voxblox::TsdfVoxel voxel;
    if (tsdf_interpolator.getVoxel(mesh_vertex_coordinates, &voxel, true)) {
      DCHECK_LE(voxel.distance, 1e-2 * tsdf_map_->voxel_size());

      // Store the isosurface vertex
      RegistrationPoint isosurface_vertex{mesh_vertex_coordinates,
                                          voxel.distance, voxel.weight};
      isosurface_vertices_.addItem(isosurface_vertex, voxel.weight);

      // Store the isosurface block
      isosurface_blocks_.emplace(
          getTsdfMap().getTsdfLayer().computeBlockIndexFromCoordinates(
              mesh_vertex_coordinates));
    }
  }
}

bool VoxgraphSubmap::overlapsWith(const VoxgraphSubmap& other_submap) const {
  // We start with a quick AABB overlap test, to discard submap pairs
  // that definitely don't overlap
  const BoundingBox aabb = getOdomFrameSurfaceAabb();
  const BoundingBox other_aabb = other_submap.getOdomFrameSurfaceAabb();
  if (!aabb.overlapsWith(other_aabb)) {
    return false;
  }

  // Next, we refine our overlap test by checking if at least one block on the
  // current submap's surface has a correspondence in the other submap
  const voxblox::Transformation T_other_submap__current_submap =
      other_submap.getPose().inverse() * getPose();
  for (const voxblox::BlockIndex& block_index : isosurface_blocks_) {
    const voxblox::Point t_current_submap__block =
        voxblox::getCenterPointFromGridIndex(block_index, block_size());
    const voxblox::Point t_other_submap__block =
        T_other_submap__current_submap * t_current_submap__block;
    const voxblox::BlockIndex other_block_index =
        voxblox::getGridIndexFromPoint<voxblox::BlockIndex>(
            t_other_submap__block,
            other_submap.getTsdfMap().getTsdfLayer().block_size_inv());
    if (other_submap.getTsdfMap().getTsdfLayer().hasBlock(other_block_index)) {
      return true;
    }
  }

  return false;
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
    double min_voxel_weight = config_.registration_filter.min_voxel_weight;
    double max_voxel_distance = config_.registration_filter.max_voxel_distance;
    for (const voxblox::BlockIndex& block_index : block_list) {
      const voxblox::Block<voxblox::TsdfVoxel>& block =
          tsdf_map_->getTsdfLayer().getBlockByIndex(block_index);
      // Iterate over all voxels in block
      for (size_t linear_index = 0u; linear_index < num_voxels_per_block;
           ++linear_index) {
        const voxblox::TsdfVoxel& voxel =
            block.getVoxelByLinearIndex(linear_index);
        // Select the observed voxels within the truncation band
        if (voxel.weight > min_voxel_weight &&
            std::abs(voxel.distance) < max_voxel_distance) {
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
    for (const voxblox::BlockIndex& block_index : block_list) {
      // Update the min and max points of the OBB
      voxblox::Point block_coordinates =
          voxblox::getCenterPointFromGridIndex(block_index, block_size());
      map_obb_.min = map_obb_.min.cwiseMin(block_coordinates - half_block_size);
      map_obb_.max = map_obb_.max.cwiseMax(block_coordinates + half_block_size);
    }
  }
  return map_obb_;
}

const BoxCornerMatrix VoxgraphSubmap::getOdomFrameSurfaceObbCorners() const {
  // Create a matrix whose columns are the box corners' homogeneous coordinates
  HomogBoxCornerMatrix box_corner_matrix = HomogBoxCornerMatrix::Constant(1);
  box_corner_matrix.topLeftCorner(3, 8) =
      getSubmapFrameSurfaceObb().getCornerCoordinates();
  // Transform the box corner coordinates to odom frame
  box_corner_matrix = getPose().getTransformationMatrix() * box_corner_matrix;
  return box_corner_matrix.topLeftCorner(3, 8);
}

const BoxCornerMatrix VoxgraphSubmap::getOdomFrameSubmapObbCorners() const {
  // Create a matrix whose columns are the box corners' homogeneous coordinates
  HomogBoxCornerMatrix box_corner_matrix = HomogBoxCornerMatrix::Constant(1);
  box_corner_matrix.topLeftCorner(3, 8) =
      getSubmapFrameSubmapObb().getCornerCoordinates();
  // Transform the box corner coordinates to odom frame
  box_corner_matrix = getPose().getTransformationMatrix() * box_corner_matrix;
  return box_corner_matrix.topLeftCorner(3, 8);
}

const BoundingBox VoxgraphSubmap::getOdomFrameSurfaceAabb() const {
  // Return the Axis Aligned Bounding Box around the isosurface in odom frame
  return BoundingBox::getAabbFromObbAndPose(getSubmapFrameSurfaceObb(),
                                            getPose());
}

const BoundingBox VoxgraphSubmap::getOdomFrameSubmapAabb() const {
  // Return the Axis Aligned Bounding Box around the full submap in odom
  // frame
  return BoundingBox::getAabbFromObbAndPose(getSubmapFrameSubmapObb(),
                                            getPose());
}

const BoxCornerMatrix VoxgraphSubmap::getOdomFrameSurfaceAabbCorners() const {
  // Return a matrix whose columns are the corner points
  // of the submap's surface AABB
  return getOdomFrameSurfaceAabb().getCornerCoordinates();
}

const BoxCornerMatrix VoxgraphSubmap::getOdomFrameSubmapAabbCorners() const {
  // Return a matrix whose columns are the corner points
  // of the full submap's AABB
  return getOdomFrameSubmapAabb().getCornerCoordinates();
}

VoxgraphSubmap::Ptr VoxgraphSubmap::LoadFromStream(
    const Config& config, std::fstream* proto_file_ptr,
    uint64_t* tmp_byte_offset_ptr) {
  // Note(alexmillane): There is no difference (for now) between the information
  // loaded in a VoxgraphSubmap and a TsdfEsdfSubmap. Therefore we just load the
  // later and copy over the data.
  TsdfEsdfSubmap::Ptr tsdf_esdf_submap = TsdfEsdfSubmap::LoadFromStream(
      config, proto_file_ptr, tmp_byte_offset_ptr);
  if (tsdf_esdf_submap == nullptr) {
    return nullptr;
  }
  // Copying over
  auto submap_ptr = std::make_shared<VoxgraphSubmap>(
      tsdf_esdf_submap->getPose(), tsdf_esdf_submap->getID(), config);
  submap_ptr->esdf_map_ = tsdf_esdf_submap->getEsdfMapPtr();
  submap_ptr->tsdf_map_ = tsdf_esdf_submap->getTsdfMapPtr();
  return submap_ptr;
}

}  // namespace voxgraph
