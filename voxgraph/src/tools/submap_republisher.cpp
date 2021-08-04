#include "voxgraph/tools/submap_republisher.h"

#include <string>

#include <minkindr_conversions/kindr_msg.h>
#include <voxblox/io/submap_io.h>
#include <voxblox_msgs/Submap.h>

#include "voxblox_ros/conversions.h"

namespace voxgraph {

SubmapRepublisher::SubmapRepublisher(const ros::NodeHandle& nh,
                                     const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {
  new_submap_notification_sub_ =
      nh_.subscribe("new_submap_written_to_disk", 1000,
                    &SubmapRepublisher::newSubmapNotificationCallback, this);
  submap_pub_ =
      nh_private_.advertise<voxblox_msgs::Submap>("submap_out", 1, false);
}

void SubmapRepublisher::newSubmapNotificationCallback(
    const transfolder_msgs::RobotSubfoldersArray& msg) {
  // Iterate over all robots
  for (const transfolder_msgs::RobotSubfolders& robot_subfolders :
       msg.robots_with_subfolders) {
    const std::string& transfolder_robot_name = robot_subfolders.robot_name;

    // Iterate over all received subfolders
    for (const std::string& absolute_subfolder_path :
         robot_subfolders.absolute_subfolder_paths) {
      // Skip subfolders that have already been processed
      if (!processed_subfolder_paths_.count(absolute_subfolder_path)) {
        // Only import voxblox submaps
        const std::string kVoxbloxSubmapPathIdentifier = "voxblox";
        if (absolute_subfolder_path.find(kVoxbloxSubmapPathIdentifier) !=
            std::string::npos) {
          // Load the submap
          voxblox::Layer<voxblox::TsdfVoxel>::Ptr tsdf_layer_ptr;
          voxblox::io::Trajectory trajectory;
          if (voxblox::io::LoadSubmap(absolute_subfolder_path, &tsdf_layer_ptr,
                                      &trajectory)) {
            // Print a warning if the robot name given by transfolder
            // and voxblox do not match. Default to name from transfolder.
            const std::string& voxblox_robot_name = trajectory.robot_name;
            if (transfolder_robot_name != voxblox_robot_name) {
              ROS_WARN_STREAM(
                  "Robot names given by transfolder and voxblox do not match ('"
                  << transfolder_robot_name << "' vs '" << voxblox_robot_name
                  << "'. Transfolder robot name ('" << transfolder_robot_name
                  << "') will be used.");
              trajectory.robot_name = transfolder_robot_name;
            }

            // Populate the submap msg and publish it
            publishSubmap(*tsdf_layer_ptr, trajectory);
          } else {
            ROS_ERROR_STREAM("Failed to load submap from directory \""
                             << absolute_subfolder_path << "\".");
          }
        }

        // Remember the subfolder to avoid processing it again
        processed_subfolder_paths_.emplace(absolute_subfolder_path);
      }
    }
  }
}

void SubmapRepublisher::publishSubmap(
    const voxblox::Layer<voxblox::TsdfVoxel>& tsdf_layer,
    const voxblox::io::Trajectory& trajectory) {
  // Publish the submap if anyone is listening
  if (0 < submap_pub_.getNumSubscribers()) {
    voxblox_msgs::Submap submap_msg;
    submap_msg.robot_name = trajectory.robot_name;

    constexpr bool kSerializeOnlyUpdated = false;
    voxblox::serializeLayerAsMsg<voxblox::TsdfVoxel>(
        tsdf_layer, kSerializeOnlyUpdated, &submap_msg.layer);

    for (const voxblox::io::StampedPose& stamped_pose :
         trajectory.stamped_poses) {
      geometry_msgs::PoseStamped pose_msg;
      pose_msg.header.frame_id = trajectory.frame_id;
      pose_msg.header.stamp.fromNSec(stamped_pose.timestamp);
      tf::poseKindrToMsg(stamped_pose.pose.cast<double>(), &pose_msg.pose);
      submap_msg.trajectory.poses.emplace_back(pose_msg);
    }

    submap_pub_.publish(submap_msg);
  }
}

}  // namespace voxgraph
