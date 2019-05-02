//
// Created by victor on 09.04.19.
//

#ifndef VOXGRAPH_FRONTEND_POSE_GRAPH_INTERFACE_H_
#define VOXGRAPH_FRONTEND_POSE_GRAPH_INTERFACE_H_

#include <voxgraph/tools/visualization/submap_visuals.h>
#include <utility>
#include "voxgraph/backend/pose_graph.h"
#include "voxgraph/common.h"
#include "voxgraph/frontend/measurement_templates.h"
#include "voxgraph/frontend/node_templates.h"
#include "voxgraph/frontend/submap_collection/voxgraph_submap_collection.h"
#include "voxgraph/tools/visualization/pose_graph_visuals.h"

namespace voxgraph {
class PoseGraphInterface {
 public:
  explicit PoseGraphInterface(
      ros::NodeHandle node_handle,
      VoxgraphSubmapCollection::Ptr submap_collection_ptr,
      bool verbose = false);

  void setVerbosity(bool verbose) { verbose_ = verbose; }
  void setMeasurementConfigFromRosParams(const ros::NodeHandle &node_handle) {
    measurement_templates_.setFromRosParams(node_handle);
  }

  void addSubmap(SubmapID submap_id, bool add_easy_odometry = false);

  // Method to recalculate which submaps overlap and update their
  // registration constraints accordingly
  void updateRegistrationConstraints();

  void addOdometryMeasurement() {}
  void addLoopClosureMeasurement() {}
  void addGpsMeasurement() {}
  void addHeightMeasurement(const SubmapID &submap_id, const double &height);

  void optimize();

  void updateSubmapCollectionPoses();

 private:
  bool verbose_;

  VoxgraphSubmapCollection::Ptr submap_collection_ptr_;

  // Pose graph and visualization tools
  PoseGraph pose_graph_;
  SubmapVisuals submap_vis_;
  PoseGraphVisuals pose_graph_vis_;
  ros::Publisher pose_graph_pub_;
  ros::Publisher submap_pub_;

  // Node and measurement config templates
  NodeTemplates node_templates_;
  MeasurementTemplates measurement_templates_;

  // Helper to add reference frames to the pose graph
  void addReferenceFrameIfMissing(ReferenceFrameNode::FrameId frame_id);
};
}  // namespace voxgraph

#endif  // VOXGRAPH_FRONTEND_POSE_GRAPH_INTERFACE_H_
