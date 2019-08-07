#ifndef VOXGRAPH_FRONTEND_MAP_TRACKER_MAP_TRACKER_H_
#define VOXGRAPH_FRONTEND_MAP_TRACKER_MAP_TRACKER_H_

#include <nav_msgs/Odometry.h>
#include <voxblox_ros/transformer.h>
#include <utility>
#include "voxgraph/common.h"
#include "voxgraph/frontend/frame_names.h"
#include "voxgraph/frontend/map_tracker/scan_to_map_registerer.h"
#include "voxgraph/frontend/submap_collection/voxgraph_submap_collection.h"

namespace voxgraph {
class MapTracker {
 public:
  explicit MapTracker(VoxgraphSubmapCollection::ConstPtr submap_collection_ptr,
                      FrameNames frame_names,
                      // TODO(victorr): Find a better solution than passing nhs
                      ros::NodeHandle nh, ros::NodeHandle nh_private,
                      bool verbose = false)
      : scan_to_map_registerer_(std::move(submap_collection_ptr), verbose),
        frame_names_(std::move(frame_names)),
        transformer_(nh, nh_private),
        verbose_(verbose) {
    //    // Publish the initial transform from the world to the drifting odom
    //    frame TfHelper::publishTransform(voxblox::Transformation(),
    //    world_frame_,
    //                               odom_frame_corrected_, true);
  }

  // TODO(victorr): Add odometry callback
  void registerPointcloud(const sensor_msgs::PointCloud2::Ptr &pointcloud_msg);

  // TODO(victorr): Fold this old method into the new structure
  // Transform lookup method that sleeps and retries a few times if the TF from
  // the base to the odom frame is not immediately available
  bool lookup_T_odom_base_link(ros::Time timestamp,
                               Transformation *T_odom_robot);

  // NOTE: See frontend/frame_names.h for frame naming convention details
  Transformation getT_mission_sensor();

  void publishTFs(const ros::Time &timestamp);

  void setVerbosity(bool verbose) { verbose_ = verbose; }
  const FrameNames &getFrameNames() const { return frame_names_; }

 private:
  bool verbose_;

  // Coordinate frame names
  // NOTE: This class is used to translate frame names between voxgraph
  //       and the other ROS nodes
  FrameNames frame_names_;

  // Transforms used to aggregate the incremental corrections
  Transformation T_M_L_;
  Transformation T_L_O_;

  // Transform from the pointcloud sensor frame to the robot's base_link
  Transformation T_B_C_;

  // Voxblox transformer used to lookup transforms from the TF tree or rosparams
  voxblox::Transformer transformer_;

  Transformation T_world_odom_corrected_;
  Transformation T_robot_sensor_;  // This transform is static

  ScanToMapRegisterer scan_to_map_registerer_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_FRONTEND_MAP_TRACKER_MAP_TRACKER_H_
