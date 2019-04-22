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
        verbose_(verbose) {
    // Zero initialize all information matrices
    odometry_information_matrix_.setZero();
    loop_closure_information_matrix_.setZero();
    gps_information_matrix_.setZero();
    height_information_matrix_.setZero();
    registration_information_matrix_.setZero();
  }

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
  PoseGraph pose_graph_;

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
