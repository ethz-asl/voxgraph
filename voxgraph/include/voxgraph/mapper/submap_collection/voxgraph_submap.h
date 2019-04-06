//
// Created by victor on 08.01.19.
//

#ifndef VOXGRAPH_MAPPER_SUBMAP_COLLECTION_VOXGRAPH_SUBMAP_H_
#define VOXGRAPH_MAPPER_SUBMAP_COLLECTION_VOXGRAPH_SUBMAP_H_

#include <cblox/core/tsdf_esdf_submap.h>
#include <ros/ros.h>
#include <map>
#include <memory>
#include "voxgraph/mapper/submap_collection/bounding_box.h"

namespace voxgraph {
class VoxgraphSubmap : public cblox::TsdfEsdfSubmap {
 public:
  typedef std::shared_ptr<VoxgraphSubmap> Ptr;
  typedef std::shared_ptr<const VoxgraphSubmap> ConstPtr;

  VoxgraphSubmap(const voxblox::Transformation &T_M_S,
                 cblox::SubmapID submap_id, Config config)
      : cblox::TsdfEsdfSubmap(T_M_S, submap_id, config) {}

  // Bounding Boxes used to check for overlap
  const BoundingBox getSubmapFrameSurfaceObb() const;
  const BoundingBox getSubmapFrameSubmapObb() const;
  const BoundingBox getWorldFrameSurfaceAabb() const;
  const BoundingBox getWorldFrameSubmapAabb() const;
  const BoxCornerMatrix getWorldFrameSurfaceObbCorners() const;
  const BoxCornerMatrix getWorldFrameSubmapObbCorners() const;
  const BoxCornerMatrix getWorldFrameSurfaceAabbCorners() const;
  const BoxCornerMatrix getWorldFrameSubmapAabbCorners() const;

  bool overlapsWith(const VoxgraphSubmap &otherSubmap) const;

  void addPoseToHistory(ros::Time timestamp,
                        voxblox::Transformation T_submap_sensor) {
    pose_history_.emplace(timestamp, T_submap_sensor);
  }

  // TODO(victorr): Move RelevantVoxelList from registration_cost_function here

 private:
  // Oriented Bounding Boxes in submap frame
  mutable BoundingBox surface_obb_;  // around the isosurface
  mutable BoundingBox map_obb_;      // around the entire submap

  typedef Eigen::Matrix<voxblox::FloatingPoint, 4, 8> HomogBoxCornerMatrix;

  // History of how the robot moved through the submap
  std::map<ros::Time, voxblox::Transformation> pose_history_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_MAPPER_SUBMAP_COLLECTION_VOXGRAPH_SUBMAP_H_
