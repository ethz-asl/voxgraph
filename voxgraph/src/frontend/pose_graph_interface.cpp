//
// Created by victor on 09.04.19.
//

#include "voxgraph/frontend/pose_graph_interface.h"
#include <utility>
#include <vector>

namespace voxgraph {
PoseGraphInterface::PoseGraphInterface(
    ros::NodeHandle node_handle,
    VoxgraphSubmapCollection::Ptr submap_collection_ptr, bool verbose)
    : submap_collection_ptr_(std::move(submap_collection_ptr)),
      submap_vis_(submap_collection_ptr_->getConfig()),
      verbose_(verbose) {
  // Zero initialize all information matrices
  odometry_information_matrix_.setZero();
  loop_closure_information_matrix_.setZero();
  gps_information_matrix_.setZero();
  height_information_matrix_.setZero();
  registration_information_matrix_.setZero();

  // Advertise the pose graph visuals publisher
  pose_graph_pub_ = node_handle.advertise<visualization_msgs::Marker>(
      "pose_graph", 100, true);
  submap_pub_ = node_handle.advertise<visualization_msgs::Marker>("submap_info",
                                                                  100, true);
}

void PoseGraphInterface::setPoseGraphConfigFromRosParams(
    const ros::NodeHandle &node_handle) {
  if (node_handle.hasParam("odometry")) {
    setInformationMatrixFromRosParams(
        ros::NodeHandle(node_handle, "odometry/information_matrix"),
        &odometry_information_matrix_);
    ROS_INFO_STREAM_COND(verbose_, "Setting odometry information matrix to:\n"
                                       << odometry_information_matrix_);
  }

  if (node_handle.hasParam("loop_closure")) {
    setInformationMatrixFromRosParams(
        ros::NodeHandle(node_handle, "loop_closure/information_matrix"),
        &loop_closure_information_matrix_);
    ROS_INFO_STREAM_COND(verbose_,
                         "Setting loop closure information matrix "
                         "to:\n"
                             << loop_closure_information_matrix_);
  }

  if (node_handle.hasParam("gps")) {
    setInformationMatrixFromRosParams(
        ros::NodeHandle(node_handle, "gps/information_matrix"),
        &gps_information_matrix_);
    ROS_INFO_STREAM_COND(verbose_,
                         "Setting gps measurement information matrix "
                         "to:\n"
                             << gps_information_matrix_);
  }

  if (node_handle.hasParam("height")) {
    height_information_matrix_.setZero();
    node_handle.param("height/information_zz", height_information_matrix_(2, 2),
                      0.0);
    ROS_INFO_STREAM_COND(verbose_,
                         "Setting height measurement information "
                         "matrix to:\n"
                             << height_information_matrix_);
  }

  if (node_handle.hasParam("submap_registration")) {
    setInformationMatrixFromRosParams(
        ros::NodeHandle(node_handle, "submap_registration/information_matrix"),
        &registration_information_matrix_);
    ROS_INFO_STREAM_COND(verbose_,
                         "Setting submap registration information "
                         "matrix to:\n"
                             << registration_information_matrix_);
  }
}

void PoseGraphInterface::setInformationMatrixFromRosParams(
    const ros::NodeHandle &node_handle,
    Constraint::InformationMatrix *information_matrix) {
  CHECK_NOTNULL(information_matrix);
  Constraint::InformationMatrix &information_matrix_ref = *information_matrix;

  // Set the upper triangular part of the information matrix from ROS params
  node_handle.param("x_x", information_matrix_ref(0, 0), 0.0);
  node_handle.param("x_y", information_matrix_ref(0, 1), 0.0);
  node_handle.param("x_z", information_matrix_ref(0, 2), 0.0);
  node_handle.param("x_yaw", information_matrix_ref(0, 3), 0.0);

  node_handle.param("y_y", information_matrix_ref(1, 1), 0.0);
  node_handle.param("y_z", information_matrix_ref(1, 2), 0.0);
  node_handle.param("y_yaw", information_matrix_ref(1, 3), 0.0);

  node_handle.param("z_z", information_matrix_ref(2, 2), 0.0);
  node_handle.param("z_yaw", information_matrix_ref(2, 3), 0.0);

  node_handle.param("yaw_yaw", information_matrix_ref(3, 3), 0.0);

  // Copy the upper to the lower triangular part, to get a symmetric info matrix
  information_matrix_ref =
      information_matrix_ref.selfadjointView<Eigen::Upper>();
}

void PoseGraphInterface::addSubmap(SubmapID submap_id, bool add_easy_odometry) {
  // Indicate that the submap is finished s.t. its cached members are generated
  {
    VoxgraphSubmap::Ptr submap_ptr =
        submap_collection_ptr_->getSubMapPtrById(submap_id);
    CHECK_NOTNULL(submap_ptr)->finishSubmap();
  }

  // Configure the submap node and add it to the pose graph
  SubmapNode::Config submap_node_config;
  submap_node_config.submap_id = submap_id;
  CHECK(submap_collection_ptr_->getSubMapPose(
      submap_id, &submap_node_config.T_world_node_initial));
  if (submap_id == 0) {
    std::cout << "Setting pose of submap 0 to constant" << std::endl;
    submap_node_config.set_constant = true;
  } else {
    submap_node_config.set_constant = false;
  }
  pose_graph_.addSubmapNode(submap_node_config);
  if (verbose_) {
    std::cout << "Added node to graph for submap: " << submap_id << std::endl;
  }

  // Easy way to add an odometry constraint between the previous and new submap
  // NOTE: This method assumes that the current submap pose purely comes from
  //       odometry and has not yet been corrected through other means
  if (add_easy_odometry && submap_collection_ptr_->size() >= 2) {
    SubmapID previous_submap_id = submap_collection_ptr_->getPreviousSubmapId();

    // Configure the odometry constraint
    RelativePoseConstraint::Config odom_constraint_config;
    odom_constraint_config.origin_submap_id = previous_submap_id;
    odom_constraint_config.destination_submap_id = submap_id;
    // TODO(victorr): Properly tune the odometry information matrix
    odom_constraint_config.information_matrix = odometry_information_matrix_;

    // Set the relative transformation
    Transformation T_world__previous_submap;
    Transformation T_world__current_submap;
    CHECK(submap_collection_ptr_->getSubMapPose(previous_submap_id,
                                                &T_world__previous_submap));
    CHECK(submap_collection_ptr_->getSubMapPose(submap_id,
                                                &T_world__current_submap));
    odom_constraint_config.T_origin_destination =
        T_world__previous_submap.inverse() * T_world__current_submap;

    // Add the odometry constraint to the pose graph
    if (verbose_) {
      std::cout << "Adding odom constraint\n"
                << "From: " << odom_constraint_config.origin_submap_id << "\n"
                << "To: " << odom_constraint_config.destination_submap_id
                << "\n"
                << "Submap currently being built in submap collection: "
                << submap_collection_ptr_->getActiveSubMapID() << "\n"
                << "T_w_s1:\n"
                << T_world__previous_submap << "\n"
                << "yaw_w_s1:" << T_world__previous_submap.log()[5] << "\n"
                << "T_w_s2:\n"
                << T_world__current_submap << "\n"
                << "yaw_w_s2:" << T_world__current_submap.log()[5] << "\n"
                << "T_s1_s2:\n"
                << odom_constraint_config.T_origin_destination << "\n"
                << "yaw_s1_s2: "
                << odom_constraint_config.T_origin_destination.log()[5] << "\n"
                << "Information matrix\n"
                << odom_constraint_config.information_matrix << std::endl;
    }

    pose_graph_.addRelativePoseConstraint(odom_constraint_config);
  }
}

void PoseGraphInterface::addHeightMeasurement(const SubmapID &submap_id,
                                              const double &height) {
  // Add the world reference frame to the pose graph if it isn't already there
  addReferenceFrameIfMissing(kWorldFrame);

  // Add the height measurement to the pose graph,
  // as an absolute pose constraint with infinite covariance on X, Y, Yaw
  AbsolutePoseConstraint::Config constraint_config;
  constraint_config.reference_frame_id = kWorldFrame;
  constraint_config.submap_id = submap_id;
  constraint_config.T_ref_submap.getPosition().z() = height;
  constraint_config.information_matrix = height_information_matrix_;
  constraint_config.allow_semi_definite_information_matrix = true;
  pose_graph_.addAbsolutePoseConstraint(constraint_config);
}

void PoseGraphInterface::updateRegistrationConstraints() {
  // Remove the previous iteration's registration constraints
  pose_graph_.resetRegistrationConstraints();

  // Add a constraint for each overlapping submap pair
  std::vector<cblox::SubmapID> submap_ids = submap_collection_ptr_->getIDs();
  for (unsigned int i = 0; i < submap_ids.size(); i++) {
    // Get a pointer to the first submap
    cblox::SubmapID first_submap_id = submap_ids[i];
    const VoxgraphSubmap& first_submap =
        submap_collection_ptr_->getSubMap(first_submap_id);

    // Publish debug visuals
    submap_vis_.publishBox(
        first_submap.getWorldFrameSurfaceAabb().getCornerCoordinates(),
        voxblox::Color::Blue(), "odom",
        "surface_abb" + std::to_string(first_submap_id), submap_pub_);

    for (unsigned int j = i + 1; j < submap_ids.size(); j++) {
      // Get the second submap
      cblox::SubmapID second_submap_id = submap_ids[j];
      const VoxgraphSubmap& second_submap =
          submap_collection_ptr_->getSubMap(second_submap_id);

      // Check whether the first and second submap overlap
      if (first_submap.overlapsWith(second_submap)) {
        // Configure the registration constraint
        RegistrationConstraint::Config constraint_config;
        constraint_config.first_submap_id = first_submap_id;
        constraint_config.second_submap_id = second_submap_id;
        constraint_config.information_matrix = registration_information_matrix_;
        // TODO(victorr): Read this from ROS params
        constraint_config.registration.registration_point_type =
            VoxgraphSubmap::RegistrationPointType::kIsosurfacePoints;
        constraint_config.registration.sampling_ratio = 0.2;

        // Add pointers to both submaps
        constraint_config.first_submap_ptr =
            submap_collection_ptr_->getSubMapConstPtrById(first_submap_id);
        constraint_config.second_submap_ptr =
            submap_collection_ptr_->getSubMapConstPtrById(second_submap_id);

        CHECK_NOTNULL(constraint_config.first_submap_ptr);
        CHECK_NOTNULL(constraint_config.second_submap_ptr);

        // Add the constraint to the pose graph
        pose_graph_.addRegistrationConstraint(constraint_config);
      }
    }
  }
}

void PoseGraphInterface::optimize() {
  // Optimize the graph
  pose_graph_.optimize();

  // Publish debug visuals
  pose_graph_vis_.publishPoseGraph(pose_graph_, "odom", "optimized",
                                   pose_graph_pub_);
}

void PoseGraphInterface::updateSubmapCollectionPoses() {
  for (const auto &submap_pose_kv : pose_graph_.getSubmapPoses()) {
    submap_collection_ptr_->setSubMapPose(submap_pose_kv.first,
                                          submap_pose_kv.second);
  }
}

void PoseGraphInterface::addReferenceFrameIfMissing(
    PoseGraphInterface::ReferenceFrames frame) {
  switch (frame) {
    case kWorldFrame:
      if (!pose_graph_.hasReferenceFrameNode(kWorldFrame)) {
        ReferenceFrameNode::Config node_config;
        node_config.reference_frame_id = kWorldFrame;
        // Fix the frame origin at (X, Y, Z, Yaw) = (0, 0, 0, 0)
        node_config.set_constant = true;
        node_config.T_world_node_initial.setIdentity();
        pose_graph_.addReferenceFrameNode(node_config);
      }
      break;
    case kGpsFrame:
      if (!pose_graph_.hasReferenceFrameNode(kGpsFrame)) {
        ReferenceFrameNode::Config node_config;
        node_config.reference_frame_id = kGpsFrame;
        // Let the frame float freely
        node_config.set_constant = false;
        pose_graph_.addReferenceFrameNode(node_config);
      }
      break;
  }
}
}  // namespace voxgraph
