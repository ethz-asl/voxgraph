//
// Created by victor on 08.01.19.
//

#ifndef VOXGRAPH_FRONTEND_SUBMAP_COLLECTION_VOXGRAPH_SUBMAP_H_
#define VOXGRAPH_FRONTEND_SUBMAP_COLLECTION_VOXGRAPH_SUBMAP_H_

#include <cblox/core/tsdf_esdf_submap.h>
#include <ros/ros.h>
#include <map>
#include <memory>
#include <vector>
#include "voxgraph/frontend/submap_collection/bounding_box.h"
#include "voxgraph/frontend/submap_collection/registration_point.h"
#include "voxgraph/frontend/submap_collection/weighted_sampler.h"

namespace voxgraph {
class VoxgraphSubmap : public cblox::TsdfEsdfSubmap {
 public:
  typedef std::shared_ptr<VoxgraphSubmap> Ptr;
  typedef std::shared_ptr<const VoxgraphSubmap> ConstPtr;

  struct Config : cblox::TsdfEsdfSubmap::Config {
    struct RegistrationFilter {
      double min_voxel_weight = 1;
      double max_voxel_distance = 0.3;
      bool use_esdf_distance = true;
    } registration_filter;
  };

  VoxgraphSubmap(const voxblox::Transformation &T_M_S,
                 const cblox::SubmapID &submap_id, const Config &config);

  // Create a VoxgraphSubmap based on a COPY of a TsdfLayer
  VoxgraphSubmap(const voxblox::Transformation &T_M_S,
                 const cblox::SubmapID &submap_id,
                 const voxblox::Layer<voxblox::TsdfVoxel> &tsdf_layer);

  void transformSubmap(const voxblox::Transformation &T_new_old);

  void addPoseToHistory(const ros::Time &timestamp,
                        const voxblox::Transformation &T_world_robot);

  // Indicate that the submap is finished and generate all cached members
  // NOTE: These cached members are mainly used in the registration cost funcs
  void finishSubmap();

  // Setter method for the registration filter config
  // NOTE: This method is mainly useful for copy or proto constructed submaps
  void setRegistrationFilterConfig(
      const Config::RegistrationFilter &registration_filter_config);

  const ros::Time getCreationTime() const;

  // The type of registration points implemented by this submap
  enum class RegistrationPointType { kIsosurfacePoints = 0, kVoxels };
  // Getter to the registration points of a certain type
  const WeightedSampler<RegistrationPoint> &getRegistrationPoints(
      RegistrationPointType registration_point_type) const;

  // Overlap and Bounding Box related methods
  bool overlapsWith(const VoxgraphSubmap &otherSubmap) const;
  const BoundingBox getSubmapFrameSurfaceObb() const;
  const BoundingBox getSubmapFrameSubmapObb() const;
  const BoundingBox getWorldFrameSurfaceAabb() const;
  const BoundingBox getWorldFrameSubmapAabb() const;
  const BoxCornerMatrix getWorldFrameSurfaceObbCorners() const;
  const BoxCornerMatrix getWorldFrameSubmapObbCorners() const;
  const BoxCornerMatrix getWorldFrameSurfaceAabbCorners() const;
  const BoxCornerMatrix getWorldFrameSubmapAabbCorners() const;

 private:
  typedef Eigen::Matrix<voxblox::FloatingPoint, 4, 8> HomogBoxCornerMatrix;
  using TsdfVoxel = voxblox::TsdfVoxel;
  using EsdfVoxel = voxblox::EsdfVoxel;
  using TsdfLayer = voxblox::Layer<TsdfVoxel>;
  using EsdfLayer = voxblox::Layer<EsdfVoxel>;
  using TsdfBlock = voxblox::Block<TsdfVoxel>;
  using EsdfBlock = voxblox::Block<EsdfVoxel>;
  using Interpolator = voxblox::Interpolator<voxblox::TsdfVoxel>;

  Config config_;

  // Track whether this submap has been declared finished
  // and all cached values have been generated
  bool finished_ = false;

  // Oriented Bounding Boxes in submap frame
  mutable BoundingBox surface_obb_;  // around the isosurface
  mutable BoundingBox map_obb_;      // around the entire submap

  // Object containing all voxels that fall within the truncation distance
  // and have a sufficiently high weight
  WeightedSampler<RegistrationPoint> relevant_voxels_;
  void findRelevantVoxelIndices();

  // Object containing all isosurface vertices stored as [x, y, z, weight]
  WeightedSampler<RegistrationPoint> isosurface_vertices_;
  void findIsosurfaceVertices();

  // History of how the robot moved through the submap
  std::map<ros::Time, voxblox::Transformation> pose_history_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_FRONTEND_SUBMAP_COLLECTION_VOXGRAPH_SUBMAP_H_
