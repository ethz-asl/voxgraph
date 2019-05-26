#include "voxgraph/frontend/submap_collection/voxgraph_submap_collection.h"
#include <utility>
#include "voxgraph/tools/tf_helper.h"

namespace voxgraph {
bool VoxgraphSubmapCollection::shouldCreateNewSubmap(
    const ros::Time &current_time) {
  if (empty()) {
    ROS_INFO_COND(verbose_,
                  "Submap collection is empty."
                  "Should create first submap");
    return true;
  } else {
    // TODO(victorr): Add options to also consider distance traveled etc
    ros::Time new_submap_creation_deadline =
        getActiveSubMap().getCreationTime() + submap_creation_interval_;
    ROS_INFO_STREAM_COND(verbose_,
                         "Current time: " << current_time << "\n"
                                          << "New creation submap deadline: "
                                          << new_submap_creation_deadline);
    return current_time > new_submap_creation_deadline;
  }
}

// Creates a gravity aligned new submap
void VoxgraphSubmapCollection::createNewSubmap(
    const Transformation &T_world_robot, const ros::Time &timestamp) {
  // Define the new submap frame to be at the current robot pose
  // and have its Z-axis aligned with gravity
  Transformation::Vector6 T_vec = T_world_robot.log();
  ROS_WARN_STREAM_COND(
      T_vec[3] > 0.05,
      "New submap creation called with proposed roll: "
          << T_vec[3] << "[rad]."
          << "Note that this angle is ignored since we work in XYZ+Yaw only. "
             "Please"
          << " provide submap poses whose Z-axis is roughly gravity aligned.");
  ROS_WARN_STREAM_COND(
      T_vec[4] > 0.05,
      "New submap creation called with proposed pitch: "
          << T_vec[4] << "[rad]."
          << "Note that this angle is ignored since we work in XYZ+Yaw only. "
             "Please"
          << " provide submap poses whose Z-axis is roughly gravity aligned.");
  T_vec[3] = 0;  // Set roll to zero
  T_vec[4] = 0;  // Set pitch to zero
  Transformation T_world__new_submap = Transformation::exp(T_vec);

  // Create the new submap
  SubmapID new_submap_id =
      cblox::SubmapCollection<VoxgraphSubmap>::createNewSubMap(
          T_world__new_submap);

  ROS_INFO_STREAM("Created submap: " << new_submap_id << " with pose\n"
                                     << T_world__new_submap);

  // Add the new submap to the timeline
  submap_timeline_.addNextSubmap(timestamp, new_submap_id);
}

VoxgraphSubmapCollection::PoseStampedVector
VoxgraphSubmapCollection::getPoseHistory() const {
  PoseStampedVector poses;
  // Iterate over all submaps
  for (VoxgraphSubmap::ConstPtr submap_ptr : getSubMaps()) {
    // Iterate over all poses in the submap
    for (const std::pair<const ros::Time, Transformation> &time_pose_pair :
         submap_ptr->getPoseHistory()) {
      geometry_msgs::PoseStamped pose_stamped_msg;
      pose_stamped_msg.header.stamp = time_pose_pair.first;
      // Transform the pose from submap frame into world frame
      const Transformation pose = submap_ptr->getPose() * time_pose_pair.second;
      tf::poseKindrToMsg(pose.cast<double>(), &pose_stamped_msg.pose);
      poses.emplace_back(pose_stamped_msg);
    }
  }
  return poses;
}
}  // namespace voxgraph
