//
// Created by victor on 09.04.19.
//

#ifndef VOXGRAPH_FRONTEND_POSE_GRAPH_INTERFACE_H_
#define VOXGRAPH_FRONTEND_POSE_GRAPH_INTERFACE_H_

#include <voxgraph/tools/visualization/submap_visuals.h>
#include <utility>
#include "voxgraph/backend/pose_graph.h"
#include "voxgraph/common.h"
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

  void setPoseGraphConfigFromRosParams(const ros::NodeHandle &node_handle);

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

  // Information matrices for each measurement type
  Constraint::InformationMatrix odometry_information_matrix_;
  Constraint::InformationMatrix loop_closure_information_matrix_;
  Constraint::InformationMatrix gps_information_matrix_;
  Constraint::InformationMatrix height_information_matrix_;
  Constraint::InformationMatrix registration_information_matrix_;
  void setInformationMatrixFromRosParams(
      const ros::NodeHandle &node_handle,
      Constraint::InformationMatrix *information_matrix);

  // Reference frames used for absolute pose constraints
  enum ReferenceFrames : ReferenceFrameNode::FrameId { kWorldFrame, kGpsFrame };
  void addReferenceFrameIfMissing(ReferenceFrames frame);
};
}  // namespace voxgraph

#endif  // VOXGRAPH_FRONTEND_POSE_GRAPH_INTERFACE_H_
