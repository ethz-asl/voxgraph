#include "voxgraph/frontend/submap_collection/voxgraph_submap_collection.h"

#include <math.h>

#include <utility>

#include "voxgraph/tools/tf_helper.h"

namespace voxgraph {
bool VoxgraphSubmapCollection::shouldCreateNewSubmap(
    const ros::Time& current_time) {
  if (empty()) {
    ROS_INFO_COND(verbose_,
                  "Submap collection is empty. "
                  "Should create first submap.");
    return true;
  } else {
    // TODO(victorr): Add options to also consider distance traveled etc
    ros::Time new_submap_creation_deadline =
        getActiveSubmap().getStartTime() + submap_creation_interval_;
    ROS_INFO_STREAM_COND(verbose_,
                         "Current time: " << current_time << "\n"
                                          << "New creation submap deadline: "
                                          << new_submap_creation_deadline);
    return current_time > new_submap_creation_deadline;
  }
}

// Creates a gravity aligned new submap
void VoxgraphSubmapCollection::createNewSubmap(
    const Transformation& T_odom_base, const ros::Time& submap_start_time) {
  // Define the new submap frame to be at the current robot pose
  // and have its Z-axis aligned with gravity
  Transformation T_odom__new_submap = gravityAlignPose(T_odom_base);

  // Create the new submap
  SubmapID new_submap_id =
      cblox::SubmapCollection<VoxgraphSubmap>::createNewSubmap(
          T_odom__new_submap);

  ROS_INFO_STREAM("Created submap: " << new_submap_id << " with pose\n"
                                     << T_odom__new_submap);

  // Add the new submap to the timeline
  submap_timeline_.addNextSubmap(submap_start_time, new_submap_id);
}

void VoxgraphSubmapCollection::addSubmap(const VoxgraphSubmap& submap) {
  addSubmapToTimeline(submap);
  cblox::SubmapCollection<VoxgraphSubmap>::addSubmap(submap);
}

void VoxgraphSubmapCollection::addSubmap(VoxgraphSubmap&& submap) {
  addSubmapToTimeline(submap);
  cblox::SubmapCollection<VoxgraphSubmap>::addSubmap(std::move(submap));
}

void VoxgraphSubmapCollection::addSubmap(const VoxgraphSubmap::Ptr submap) {
  addSubmapToTimeline(*submap);
  cblox::SubmapCollection<VoxgraphSubmap>::addSubmap(submap);
}

void VoxgraphSubmapCollection::addSubmapToTimeline(
    const VoxgraphSubmap& submap) {
  ROS_INFO_STREAM("Created submap " << submap.getID() << " with pose\n"
                                    << submap.getPose());
  // Add the new submap to the timeline
  submap_timeline_.addNextSubmap(submap.getEndTime(), submap.getID());
}

void VoxgraphSubmapCollection::createNewSubmap(const Transformation& T_O_S,
                                               const SubmapID submap_id) {
  ROS_ERROR_STREAM(
      "Used createNewSubmap(...) method from cblox::SubmapCollection "
      "base class. Submaps that are added this way cannot be included in the "
      "submap timeline due to missing time information. Please use the "
      "derived VoxgraphSubmapCollection::createNewSubmap methods instead.");
  cblox::SubmapCollection<VoxgraphSubmap>::createNewSubmap(T_O_S, submap_id);
}

SubmapID VoxgraphSubmapCollection::createNewSubmap(
    const Transformation& T_O_S) {
  ROS_ERROR_STREAM(
      "Used createNewSubmap(...) method from cblox::SubmapCollection "
      "base class. Submaps that are added this way cannot be included in the "
      "submap timeline due to missing time information. Please use the "
      "derived VoxgraphSubmapCollection::createNewSubmap methods instead.");
  return cblox::SubmapCollection<VoxgraphSubmap>::createNewSubmap(T_O_S);
}

bool VoxgraphSubmapCollection::lookupActiveSubmapByTime(
    const ros::Time& timestamp, SubmapID* submap_id) {
  if (submap_timeline_.lookupActiveSubmapByTime(timestamp, submap_id)) {
    return getSubmap(*submap_id).getStartTime() <= timestamp;
  }
  return false;
}

VoxgraphSubmapCollection::PoseStampedVector
VoxgraphSubmapCollection::getPoseHistory() const {
  using PoseCountPair = std::pair<Transformation, int>;
  std::map<ros::Time, PoseCountPair> averaged_trajectory;
  // Iterate over all submaps and poses
  for (const VoxgraphSubmap::ConstPtr& submap_ptr : getSubmapConstPtrs()) {
    for (const std::pair<const ros::Time, Transformation>& time_pose_pair :
         submap_ptr->getPoseHistory()) {
      // Transform the pose from submap frame into odom frame
      const Transformation T_O_B_i =
          submap_ptr->getPose() * time_pose_pair.second;
      const ros::Time& timestamp_i = time_pose_pair.first;

      // Insert, or average if there was a previous pose with the same stamp
      auto it = averaged_trajectory.find(timestamp_i);
      if (it == averaged_trajectory.end()) {
        averaged_trajectory.emplace(timestamp_i, PoseCountPair(T_O_B_i, 1));
      } else {
        it->second.second++;
        const double lambda = 1.0 / it->second.second;
        it->second.first = kindr::minimal::interpolateComponentwise(
            it->second.first, T_O_B_i, lambda);
      }
    }
  }

  // Copy the averaged trajectory poses into the msg and compute the total
  // trajectory length
  PoseStampedVector poses;
  float total_trajectory_length = 0.0;
  Transformation::Position previous_position;
  bool previous_position_initialized = false;
  for (const auto& kv : averaged_trajectory) {
    geometry_msgs::PoseStamped pose_stamped_msg;
    pose_stamped_msg.header.stamp = kv.first;
    tf::poseKindrToMsg(kv.second.first.cast<double>(), &pose_stamped_msg.pose);
    poses.emplace_back(pose_stamped_msg);

    if (previous_position_initialized) {
      total_trajectory_length +=
          (kv.second.first.getPosition() - previous_position).norm();
    } else {
      previous_position_initialized = true;
    }
    previous_position = kv.second.first.getPosition();
  }
  ROS_INFO_STREAM("Total trajectory length: " << total_trajectory_length);
  return poses;
}

Transformation VoxgraphSubmapCollection::gravityAlignPose(
    const Transformation& input_pose) {
  // Use the logarithmic map to get the pose's [x, y, z, r, p, y] components
  Transformation::Vector6 T_vec = input_pose.log();

  // Print a warning if the original pitch & roll components were large
  constexpr float angle_threshold_rad = 30.f /* deg */ / 180.f * M_PI;
  if (std::abs(T_vec[3]) > angle_threshold_rad) {
    ROS_WARN_STREAM_THROTTLE(
        1, "New submap creation called with proposed roll: "
               << T_vec[3]
               << "[rad]. This most likely isn't problematic, but keep in mind "
                  "that voxgraph only optimizes over XYZ and Yaw. So please "
                  "make sure that voxgraph's odometry input is in a coordinate "
                  "system whose Z-axis is roughly gravity aligned.");
  }
  if (std::abs(T_vec[4]) > angle_threshold_rad) {
    ROS_WARN_STREAM_THROTTLE(
        1, "New submap creation called with proposed pitch: "
               << T_vec[4]
               << "[rad]. This most likely isn't problematic, but keep in mind "
                  "that voxgraph only optimizes over XYZ and Yaw. So please "
                  "make sure that voxgraph's odometry input is in a coordinate "
                  "system whose Z-axis is roughly gravity aligned.");
  }

  // Set the roll and pitch to zero
  T_vec[3] = 0;
  T_vec[4] = 0;

  // Return the gravity aligned pose as a translation + quaternion,
  // using the exponential map
  return Transformation::exp(T_vec);
}
}  // namespace voxgraph
