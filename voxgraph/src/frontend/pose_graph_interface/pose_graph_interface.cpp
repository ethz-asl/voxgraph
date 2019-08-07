#include "voxgraph/frontend/pose_graph_interface/pose_graph_interface.h"
#include <utility>
#include <vector>

namespace voxgraph {
PoseGraphInterface::PoseGraphInterface(
    ros::NodeHandle node_handle,
    VoxgraphSubmapCollection::Ptr submap_collection_ptr, bool verbose)
    : submap_collection_ptr_(std::move(submap_collection_ptr)),
      submap_vis_(submap_collection_ptr_->getConfig()),
      verbose_(verbose) {
  // Advertise the pose graph visuals publisher
  pose_graph_pub_ = node_handle.advertise<visualization_msgs::Marker>(
      "pose_graph", 100, true);
  submap_pub_ =
      node_handle.advertise<visualization_msgs::Marker>("submap_info", 1, true);
}

void PoseGraphInterface::addSubmap(SubmapID submap_id, bool add_easy_odometry) {
  // Indicate that the submap is finished s.t. its cached members are generated
  {
    VoxgraphSubmap::Ptr submap_ptr =
        submap_collection_ptr_->getSubmapPtr(submap_id);
    CHECK_NOTNULL(submap_ptr)->finishSubmap();
  }

  // Configure the submap node and add it to the pose graph
  SubmapNode::Config node_config = node_templates_.submap;
  node_config.submap_id = submap_id;
  CHECK(submap_collection_ptr_->getSubmapPose(
      submap_id, &node_config.T_mission_node_initial));
  if (submap_id == 0) {
    ROS_INFO("Setting pose of submap 0 to constant");
    node_config.set_constant = true;
  } else {
    node_config.set_constant = false;
  }
  pose_graph_.addSubmapNode(node_config);
  ROS_INFO_STREAM_COND(verbose_,
                       "Added node to graph for submap: " << submap_id);

  // TODO(victorr): Move this to the addOdometryMeasurement method
  // Easy way to add an odometry constraint between the previous and new submap
  // NOTE: This method assumes that the current submap pose purely comes from
  //       odometry and has not yet been corrected through other means
  if (add_easy_odometry && submap_collection_ptr_->size() >= 2) {
    SubmapID previous_submap_id = submap_collection_ptr_->getPreviousSubmapId();

    // Configure the odometry constraint
    RelativePoseConstraint::Config constraint_config =
        measurement_templates_.odometry;
    constraint_config.origin_submap_id = previous_submap_id;
    constraint_config.destination_submap_id = submap_id;

    // Set the relative transformation
    Transformation T_mission__previous_submap;
    Transformation T_mission__current_submap;
    CHECK(submap_collection_ptr_->getSubmapPose(previous_submap_id,
                                                &T_mission__previous_submap));
    CHECK(submap_collection_ptr_->getSubmapPose(submap_id,
                                                &T_mission__current_submap));
    constraint_config.T_origin_destination =
        T_mission__previous_submap.inverse() * T_mission__current_submap;

    // Add the odometry constraint to the pose graph
    if (verbose_) {
      std::cout << "Adding odom constraint\n"
                << "From: " << constraint_config.origin_submap_id << "\n"
                << "To: " << constraint_config.destination_submap_id << "\n"
                << "Submap currently being built in submap collection: "
                << submap_collection_ptr_->getActiveSubmapID() << "\n"
                << "T_w_s1:\n"
                << T_mission__previous_submap << "\n"
                << "yaw_w_s1:" << T_mission__previous_submap.log()[5] << "\n"
                << "T_w_s2:\n"
                << T_mission__current_submap << "\n"
                << "yaw_w_s2:" << T_mission__current_submap.log()[5] << "\n"
                << "T_s1_s2:\n"
                << constraint_config.T_origin_destination << "\n"
                << "yaw_s1_s2: "
                << constraint_config.T_origin_destination.log()[5] << "\n"
                << "Information matrix\n"
                << constraint_config.information_matrix << std::endl;
    }
    pose_graph_.addRelativePoseConstraint(constraint_config);
  }
}

void PoseGraphInterface::addHeightMeasurement(const SubmapID &submap_id,
                                              const double &height) {
  // Configure the constraint
  AbsolutePoseConstraint::Config constraint_config =
      measurement_templates_.height;
  constraint_config.submap_id = submap_id;
  constraint_config.T_ref_submap.getPosition().z() = height;

  // Add the mission reference frame to the pose graph if it isn't already there
  addReferenceFrameIfMissing(constraint_config.reference_frame_id);

  // Add the height measurement to the pose graph
  pose_graph_.addAbsolutePoseConstraint(constraint_config);
}

void PoseGraphInterface::updateOverlappingSubmapList() {
  // Clear the old overlapping submap list
  overlapping_submap_list_.clear();

  // For each submap in the collection, check which submaps it overlaps with
  std::vector<cblox::SubmapID> submap_ids = submap_collection_ptr_->getIDs();
  for (size_t i = 0; i < submap_ids.size(); i++) {
    // Get a pointer to the first submap
    cblox::SubmapID first_submap_id = submap_ids[i];
    const VoxgraphSubmap &first_submap =
        submap_collection_ptr_->getSubmap(first_submap_id);

    // Publish debug visuals
    // TODO(victorr): Move this to a better place (e.g. visualization class)
    if (submap_pub_.getNumSubscribers() > 0) {
      submap_vis_.publishBox(
          first_submap.getMissionFrameSurfaceAabb().getCornerCoordinates(),
          voxblox::Color::Blue(), "odom",
          "surface_abb" + std::to_string(first_submap_id), submap_pub_);
    }

    // Test the current submaps against all subsequent submaps
    // NOTE: Testing against the subsequent submaps only is sufficient to test
    //       all pairs in the submap collection, since the overlap test is
    //       symmetric (i.e. pair A<->B and B<->A are equivalent)
    for (size_t j = i + 1; j < submap_ids.size(); j++) {
      // Get the second submap
      cblox::SubmapID second_submap_id = submap_ids[j];
      const VoxgraphSubmap &second_submap =
          submap_collection_ptr_->getSubmap(second_submap_id);

      // Check whether the first and second submap overlap
      if (first_submap.overlapsWith(second_submap)) {
        overlapping_submap_list_.emplace_back(first_submap_id,
                                              second_submap_id);
      }
    }
  }
}

void PoseGraphInterface::updateRegistrationConstraints() {
  // Remove the previous iteration's registration constraints
  pose_graph_.resetRegistrationConstraints();

  // Redetect which submaps overlap
  updateOverlappingSubmapList();

  // Add the updated registration constraints
  for (const SubmapIdPair &submap_pair : overlapping_submap_list_) {
    // Configure the registration constraint
    RegistrationConstraint::Config constraint_config =
        measurement_templates_.registration;
    constraint_config.first_submap_id = submap_pair.first;
    constraint_config.second_submap_id = submap_pair.second;

    // Add pointers to both submaps
    constraint_config.first_submap_ptr =
        submap_collection_ptr_->getSubmapConstPtr(submap_pair.first);
    constraint_config.second_submap_ptr =
        submap_collection_ptr_->getSubmapConstPtr(submap_pair.second);
    CHECK_NOTNULL(constraint_config.first_submap_ptr);
    CHECK_NOTNULL(constraint_config.second_submap_ptr);

    // Add the constraint to the pose graph
    pose_graph_.addRegistrationConstraint(constraint_config);
  }
}

void PoseGraphInterface::optimize() {
  // Optimize the graph
  pose_graph_.optimize();

  // Publish debug visuals
  if (pose_graph_pub_.getNumSubscribers() > 0) {
    pose_graph_vis_.publishPoseGraph(pose_graph_, "odom", "optimized",
                                     pose_graph_pub_);
  }
}

void PoseGraphInterface::updateSubmapCollectionPoses() {
  for (const auto &submap_pose_kv : pose_graph_.getSubmapPoses()) {
    submap_collection_ptr_->setSubmapPose(submap_pose_kv.first,
                                          submap_pose_kv.second);
  }
}

bool PoseGraphInterface::getEdgeCovarianceMap(
    PoseGraph::EdgeCovarianceMap *edge_covariance_map_ptr) const {
  CHECK_NOTNULL(edge_covariance_map_ptr);

  // Request covariance estimates for all overlapping submap pairs
  for (const SubmapIdPair &overlapping_submap_pair : overlapping_submap_list_) {
    edge_covariance_map_ptr->emplace(overlapping_submap_pair,
                                     PoseGraph::EdgeCovarianceMatrix::Zero());
  }

  return pose_graph_.getEdgeCovarianceMap(edge_covariance_map_ptr);
}

void PoseGraphInterface::addReferenceFrameIfMissing(
    ReferenceFrameNode::FrameId frame_id) {
  if (!pose_graph_.hasReferenceFrameNode(frame_id)) {
    pose_graph_.addReferenceFrameNode(
        node_templates_.getReferenceFrameConfigById(frame_id));
  }
}
}  // namespace voxgraph
