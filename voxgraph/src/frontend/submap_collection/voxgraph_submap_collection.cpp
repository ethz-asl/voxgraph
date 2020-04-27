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
    const Transformation& T_mission_base, const ros::Time& timestamp) {
  // Define the new submap frame to be at the current robot pose
  // and have its Z-axis aligned with gravity
  Transformation T_mission__new_submap = gravityAlignPose(T_mission_base);

  // Create the new submap
  SubmapID new_submap_id =
      cblox::SubmapCollection<VoxgraphSubmap>::createNewSubmap(
          T_mission__new_submap);

  ROS_INFO_STREAM("Created submap: " << new_submap_id << " with pose\n"
                                     << T_mission__new_submap);

  // Add the new submap to the timeline
  submap_timeline_.addNextSubmap(timestamp, new_submap_id);
}

VoxgraphSubmapCollection::PoseStampedVector
VoxgraphSubmapCollection::getPoseHistory() const {
  PoseStampedVector poses;
  // Iterate over all submaps
  for (const VoxgraphSubmap::ConstPtr& submap_ptr : getSubmapConstPtrs()) {
    // Iterate over all poses in the submap
    for (const std::pair<const ros::Time, Transformation>& time_pose_pair :
         submap_ptr->getPoseHistory()) {
      geometry_msgs::PoseStamped pose_stamped_msg;
      pose_stamped_msg.header.stamp = time_pose_pair.first;
      // Transform the pose from submap frame into mission frame
      const Transformation pose = submap_ptr->getPose() * time_pose_pair.second;
      tf::poseKindrToMsg(pose.cast<double>(), &pose_stamped_msg.pose);
      poses.emplace_back(pose_stamped_msg);
    }
  }
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
