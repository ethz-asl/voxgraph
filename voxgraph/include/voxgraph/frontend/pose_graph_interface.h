//
// Created by victor on 09.04.19.
//

#ifndef VOXGRAPH_FRONTEND_POSE_GRAPH_INTERFACE_H_
#define VOXGRAPH_FRONTEND_POSE_GRAPH_INTERFACE_H_

#include <utility>
#include "voxgraph/backend/pose_graph.h"
#include "voxgraph/common.h"
#include "voxgraph/frontend/submap_collection/voxgraph_submap.h"

namespace voxgraph {
class PoseGraphInterface {
 public:
  explicit PoseGraphInterface(
      cblox::SubmapCollection<VoxgraphSubmap>::Ptr submap_collection_ptr)
      : submap_collection_ptr_(std::move(submap_collection_ptr)) {}

  void addSubmap(SubmapID submap_id);

  void addOdometryMeasurement() {}

  void addLoopClosureMeasurement() {}

  void addGpsMeasurement() {}

  void optimize();

  void updateSubmapCollectionPoses();

 private:
  cblox::SubmapCollection<VoxgraphSubmap>::Ptr submap_collection_ptr_;
  PoseGraph pose_graph_;

  void updateRegistrationConstraints();
};
}  // namespace voxgraph

#endif  // VOXGRAPH_FRONTEND_POSE_GRAPH_INTERFACE_H_
