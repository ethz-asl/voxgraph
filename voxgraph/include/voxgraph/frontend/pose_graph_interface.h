//
// Created by victor on 09.04.19.
//

#ifndef VOXGRAPH_FRONTEND_POSE_GRAPH_INTERFACE_H_
#define VOXGRAPH_FRONTEND_POSE_GRAPH_INTERFACE_H_

#include <utility>
#include "voxgraph/backend/pose_graph.h"
#include "voxgraph/common.h"
#include "voxgraph/frontend/submap_collection/voxgraph_submap_collection.h"

namespace voxgraph {
class PoseGraphInterface {
 public:
  explicit PoseGraphInterface(
      VoxgraphSubmapCollection::Ptr submap_collection_ptr, bool verbose = false)
      : submap_collection_ptr_(std::move(submap_collection_ptr)),
        verbose_(verbose) {}

  void addSubmap(SubmapID submap_id, bool add_easy_odometry = false);

  void addOdometryMeasurement() {}

  void addLoopClosureMeasurement() {}

  void addGpsMeasurement() {}

  void optimize();

  void updateSubmapCollectionPoses();

 private:
  bool verbose_;

  VoxgraphSubmapCollection::Ptr submap_collection_ptr_;
  PoseGraph pose_graph_;

  void updateRegistrationConstraints();
};
}  // namespace voxgraph

#endif  // VOXGRAPH_FRONTEND_POSE_GRAPH_INTERFACE_H_
