//
// Created by victor on 08.04.19.
//

#include "voxgraph/frontend/submap_collection/voxgraph_submap_collection.h"
#include "voxgraph/tools/tf_helper.h"

namespace voxgraph {
bool VoxgraphSubmapCollection::shouldCreateNewSubmap(
    const ros::Time &current_time) {
  // TODO(victorr): Also take the pose uncertainty etc into account
  ros::Time new_submap_creation_deadline =
      current_submap_creation_stamp_ + submap_creation_interval_;
  return current_time > new_submap_creation_deadline;
}

// Creates a gravity aligned new submap
void VoxgraphSubmapCollection::createNewSubmap(
    const Transformation &T_world_robot, const ros::Time &timestamp) {
  // Define the new submap frame to be at the current robot pose
  // and have its Z-axis aligned with gravity
  Transformation::Vector6 T_vec = T_world_robot.log();
  T_vec[3] = 0;  // Set roll to zero
  T_vec[4] = 0;  // Set pitch to zero
  Transformation T_world__new_submap = Transformation::exp(T_vec);

  // Create the new submap
  SubmapID new_submap_id =
      cblox::SubmapCollection<VoxgraphSubmap>::createNewSubMap(
          T_world__new_submap);
  current_submap_creation_stamp_ = timestamp;

  ROS_INFO_STREAM("Created submap: " << new_submap_id << " with pose\n"
                                     << T_world__new_submap);
}
}  // namespace voxgraph
