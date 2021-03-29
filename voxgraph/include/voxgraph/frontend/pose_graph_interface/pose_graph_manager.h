#ifndef VOXGRAPH_FRONTEND_POSE_GRAPH_INTERFACE_POSE_GRAPH_MANAGER_H_
#define VOXGRAPH_FRONTEND_POSE_GRAPH_INTERFACE_POSE_GRAPH_MANAGER_H_

#include <string>
#include <utility>
#include <vector>

#include "voxgraph/backend/pose_graph.h"
#include "voxgraph/common.h"
#include "voxgraph/frontend/pose_graph_interface/measurement_templates.h"
#include "voxgraph/frontend/pose_graph_interface/node_templates.h"
#include "voxgraph/frontend/submap_collection/voxgraph_submap_collection.h"
#include "voxgraph/tools/visualization/pose_graph_visuals.h"
#include "voxgraph/tools/visualization/submap_visuals.h"

namespace voxgraph {
class PoseGraphManager {
 public:
  typedef std::vector<SubmapIdPair> OverlappingSubmapList;

  explicit PoseGraphManager(ros::NodeHandle node_handle,
                            VoxgraphSubmapCollection::Ptr submap_collection_ptr,
                            voxblox::MeshIntegratorConfig mesh_config,
                            std::string visualizations_odom_frame,
                            bool verbose = false);

  void setVerbosity(bool verbose) { verbose_ = verbose; }
  void setMeasurementConfigFromRosParams(const ros::NodeHandle& node_handle) {
    measurement_templates_.setFromRosParams(node_handle);
  }
  void setRegistrationConstraintsEnabled(const bool enabled) {
    registration_constraints_enabled_ = enabled;
  }
  bool getRegistrationConstraintsEnabled() const {
    return registration_constraints_enabled_;
  }

  void addSubmap(SubmapID submap_id);

  // NOTE: The pose graph optimization works in 4D. Therefore the
  //       pitch and roll components of T_S1_S2 are simply ignored
  //       by the RelativePoseCostFunction.
  void addOdometryMeasurement(const SubmapID& first_submap_id,
                              const SubmapID& second_submap_id,
                              const Transformation& T_S1_S2);
  void addLoopClosureMeasurement(const SubmapID& from_submap,
                                 const SubmapID& to_submap,
                                 const Transformation& transform);
  void addGpsMeasurement() {}
  void addHeightMeasurement(const SubmapID& submap_id, const double& height);

  // Method to recalculate which submaps overlap and update their
  // registration constraints accordingly
  void updateSlidingPoseGraphRegistrationConstraints();
  void updateFullPoseGraphRegistrationConstraints();

  void absorbSlidingWindowIntoFullPoseGraph();

  // Optimize
  void optimizeSlidingPoseGraph();
  void optimizeFullPoseGraph();

  //  bool getEdgeCovarianceMap(
  //      PoseGraph::EdgeCovarianceMap* edge_covariance_map_ptr) const;

  const PoseGraph::SolverSummaryList& getSlidingPoseGraphSolverSummaries()
      const {
    return sliding_pose_graph_.getSolverSummaries();
  }
  const PoseGraph::SolverSummaryList& getFullPoseGraphSolverSummaries() const {
    return full_pose_graph_.getSolverSummaries();
  }

 private:
  bool verbose_;
  VoxgraphSubmapCollection::Ptr submap_collection_ptr_;

  // Pose graphs
  PoseGraph sliding_pose_graph_;
  PoseGraph full_pose_graph_;
  std::mutex pose_graph_merging_mutex_;
  void updateSubmapCollectionPosesBasedOnSlidingPoseGraph();

  // Visualization tools
  const std::string visualization_odom_frame_;
  SubmapVisuals submap_vis_;
  PoseGraphVisuals pose_graph_vis_;
  ros::Publisher pose_graph_pub_;
  ros::Publisher submap_pub_;

  // Node and measurement config templates
  NodeTemplates node_templates_;
  MeasurementTemplates measurement_templates_;
  bool registration_constraints_enabled_;

  // Registration constraint handlers
  void updateRegistrationConstraintsForOverlappingSubmapPairs(
      PoseGraph* pose_graph_ptr);
  void addRegistrationConstraintForSubmapPair(const SubmapID first_submap_id,
                                              const SubmapID second_submap_id,
                                              PoseGraph* pose_graph_ptr);

  // Helper to add reference frames to the pose graph
  void addReferenceFrameIfMissing(ReferenceFrameNode::FrameId frame_id);

  // Optimize
  void optimizeSlidingPoseGraphImpl();
};
}  // namespace voxgraph

#endif  // VOXGRAPH_FRONTEND_POSE_GRAPH_INTERFACE_POSE_GRAPH_MANAGER_H_
