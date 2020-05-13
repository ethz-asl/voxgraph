#include "voxgraph/frontend/map_tracker/map_tracker.h"

#include <limits>
#include <string>
#include <utility>

#include "voxgraph/tools/tf_helper.h"

namespace voxgraph {
MapTracker::MapTracker(VoxgraphSubmapCollection::ConstPtr submap_collection_ptr,
                       FrameNames frame_names, bool verbose)
    : verbose_(verbose),
      submap_collection_ptr_(submap_collection_ptr),
      frame_names_(std::move(frame_names)),
      tf_transformer_(),
      odom_transformer_() {}

void MapTracker::subscribeToTopics(ros::NodeHandle nh,
                                   const std::string& odometry_input_topic) {
  if (!odometry_input_topic.empty()) {
    ROS_INFO_STREAM("Using odometry from ROS topic: " << odometry_input_topic);
    use_odom_from_tfs_ = false;
    odom_transformer_.subscribeToTopic(nh, odometry_input_topic);
  }
}

void MapTracker::advertiseTopics(ros::NodeHandle nh_private,
                                 const std::string& odometry_output_topic) {}

bool MapTracker::updateToTime(const ros::Time& timestamp,
                              const std::string& sensor_frame_id) {
  // Keep track of the timestamp that the MapTracker is currently at
  current_timestamp_ = timestamp;

  // Update the odometry
  if (use_odom_from_tfs_) {
    // Update the odometry estimate
    if (!tf_transformer_.lookupTransform(frame_names_.input_odom_frame,
                                         frame_names_.input_base_link_frame,
                                         timestamp, &T_O_B_)) {
      return false;
    }
  } else {
    if (!odom_transformer_.lookupTransform(timestamp, &T_O_B_)) {
      return false;
    }
  }

  // Express the odometry pose in the frame of the current submap
  T_S_B_ = initial_T_S_O_ * T_O_B_;

  // Get the transformation from the pointcloud sensor to the robot's
  // base link from TFs, unless it was already provided through ROS params
  if (use_sensor_calibration_from_tfs_) {
    // TODO(victorr): Implement option to provide a sensor_frame_id instead of
    //                taking the one from the message
    // Strip leading slashes if needed to avoid TF errors
    std::string tf_sensor_frame_id;
    if (sensor_frame_id[0] == '/') {
      tf_sensor_frame_id = sensor_frame_id.substr(1, sensor_frame_id.length());
    } else {
      tf_sensor_frame_id = sensor_frame_id;
    }

    // Lookup the transform
    if (!tf_transformer_.lookupTransform(frame_names_.input_base_link_frame,
                                         tf_sensor_frame_id, timestamp,
                                         &T_B_C_)) {
      return false;
    }
  }

  // Signal that all transforms were successfully updated
  return true;
}

void MapTracker::switchToNewSubmap(const Transformation& T_M_S_new) {
  // Store the initial submap pose for visualization purposes
  initial_T_M_S_ = T_M_S_new;

  // Get the pose of the new submap in odom frame
  Transformation T_O_S = VoxgraphSubmapCollection::gravityAlignPose(T_O_B_);

  // Store the transform used to convert the odometry input into submap frame
  initial_T_S_O_ = T_O_S.inverse();

  // Update the current robot pose
  // NOTE: This initial pose can differ from Identity, since the submap pose
  //       has zero pitch and roll whereas the robot pose is in 6DoF
  T_S_B_ = initial_T_S_O_ * T_O_B_;
}

void MapTracker::publishTFs() {
  TfHelper::publishTransform(submap_collection_ptr_->getActiveSubmapPose(),
                             frame_names_.output_mission_frame,
                             frame_names_.output_active_submap_frame, false,
                             current_timestamp_);
  TfHelper::publishTransform(
      initial_T_S_O_, frame_names_.output_active_submap_frame,
      frame_names_.output_odom_frame, false, current_timestamp_);

  if (frame_names_.input_odom_frame != frame_names_.output_odom_frame ||
      frame_names_.input_base_link_frame !=
          frame_names_.output_base_link_frame ||
      !use_odom_from_tfs_) {
    // Republish the odometry if the output frame names are different,
    // or if the odom input is coming from a ROS topic
    // (in which case it might not yet be in the TF tree)
    TfHelper::publishTransform(T_O_B_, frame_names_.output_odom_frame,
                               frame_names_.output_base_link_frame, false,
                               current_timestamp_);
  }

  TfHelper::publishTransform(T_B_C_, frame_names_.output_base_link_frame,
                             frame_names_.output_sensor_frame, true,
                             current_timestamp_);
}

Transformation MapTracker::get_T_M_B() {
  if (submap_collection_ptr_->empty()) {
    // If no submap has been created yet, return the odometry pose
    return T_O_B_;
  } else {
    return submap_collection_ptr_->getActiveSubmapPose() * T_S_B_;
  }
}

void MapTracker::set_T_B_C(const Transformation& T_B_C) {
  T_B_C_ = T_B_C;
  use_sensor_calibration_from_tfs_ = false;
}
}  // namespace voxgraph
