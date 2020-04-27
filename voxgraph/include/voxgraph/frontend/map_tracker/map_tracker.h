#ifndef VOXGRAPH_FRONTEND_MAP_TRACKER_MAP_TRACKER_H_
#define VOXGRAPH_FRONTEND_MAP_TRACKER_MAP_TRACKER_H_

#include <string>
#include <utility>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <voxblox_ros/transformer.h>

#include "voxgraph/common.h"
#include "voxgraph/frontend/frame_names.h"
#include "voxgraph/frontend/map_tracker/transformers/odometry_transformer.h"
#include "voxgraph/frontend/map_tracker/transformers/tf_transformer.h"
#include "voxgraph/frontend/submap_collection/voxgraph_submap_collection.h"

namespace voxgraph {
class MapTracker {
 public:
  explicit MapTracker(VoxgraphSubmapCollection::ConstPtr submap_collection_ptr,
                      FrameNames frame_names, bool verbose = false);

  void subscribeToTopics(ros::NodeHandle nh,
                         const std::string& odometry_input_topic);
  void advertiseTopics(ros::NodeHandle nh_private,
                       const std::string& odometry_output_topic);

  bool updateToTime(const ros::Time& timestamp,
                    const std::string& sensor_frame_id);
  void switchToNewSubmap(const Transformation& T_M_S_new);

  void publishTFs();

  // Transform getter methods
  // NOTE: For more info on the frame naming conventions,
  //       see the FrameNames class
  Transformation get_T_M_B();
  Transformation get_T_O_B() { return T_O_B_; }
  Transformation get_T_S_B() { return T_S_B_; }
  Transformation get_T_S_C() { return T_S_B_ * T_B_C_; }

  void set_T_B_C(const Transformation& T_B_C);

  const FrameNames& getFrameNames() const { return frame_names_; }

  // Config get/setters
  void setVerbosity(bool verbose) { verbose_ = verbose; }

 private:
  bool verbose_;

  // Pointer to the submap collection containing the one that is being tracked
  VoxgraphSubmapCollection::ConstPtr submap_collection_ptr_;

  // Timestamp at which the MapTracker was last updated
  ros::Time current_timestamp_;

  // Coordinate frame names
  // NOTE: This class is used to translate frame names between voxgraph
  //       and the other ROS nodes
  FrameNames frame_names_;

  // Transform that stores the current raw odometry input
  Transformation T_O_B_;

  // Transform from the odom origin to the pose at which
  // the current submap was created
  // NOTE: This transform is used to convert the pose from the odometry input
  //       into the coordinate frame of the current submap
  Transformation initial_T_S_O_;
  Transformation initial_T_M_S_;  // for visualization purposes only

  // Transform that tracks the odometry in submap frame
  Transformation T_S_B_;

  // Transform from the pointcloud sensor frame to the robot's base_link
  Transformation T_B_C_;

  // Transformer class to lookup transforms from the TF tree or an odom topic
  bool use_odom_from_tfs_ = true;
  // NOTE: use_odom_tfs_ is automatically set to true if subscribeToTopics()
  //       is called with a non-empty odometry_input_topic argument
  bool use_sensor_calibration_from_tfs_ = true;
  // NOTE: use_sensor_calibration_from_tfs_ is automatically set to
  //       false if a calibration is provided through method set_T_B_C()
  TfTransformer tf_transformer_;
  OdometryTransformer odom_transformer_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_FRONTEND_MAP_TRACKER_MAP_TRACKER_H_
