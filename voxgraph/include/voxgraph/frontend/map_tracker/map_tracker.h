#ifndef VOXGRAPH_FRONTEND_MAP_TRACKER_MAP_TRACKER_H_
#define VOXGRAPH_FRONTEND_MAP_TRACKER_MAP_TRACKER_H_

#include <maplab_msgs/OdometryWithImuBiases.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <voxblox_ros/transformer.h>
#include <string>
#include <utility>
#include "voxgraph/common.h"
#include "voxgraph/frontend/frame_names.h"
#include "voxgraph/frontend/map_tracker/scan_to_map_registerer.h"
#include "voxgraph/frontend/map_tracker/transformers/odometry_transformer.h"
#include "voxgraph/frontend/map_tracker/transformers/tf_transformer.h"
#include "voxgraph/frontend/submap_collection/voxgraph_submap_collection.h"

namespace voxgraph {
class MapTracker {
 public:
  explicit MapTracker(VoxgraphSubmapCollection::ConstPtr submap_collection_ptr,
                      FrameNames frame_names, bool verbose = false);

  void subscribeToTopics(ros::NodeHandle nh,
                         const std::string &odometry_input_topic,
                         const std::string &imu_biases_topic);
  void imuBiasesCallback(const sensor_msgs::Imu::ConstPtr &imu_biases);
  void advertiseTopics(ros::NodeHandle nh_private,
                       const std::string &odometry_output_topic);

  bool updateToTime(const ros::Time &timestamp,
                    std::string sensor_frame_id);

  // TODO(victorr): Remove this once the robot pose is defined stored relative
  //                to the current submap instead of the mission frame
  void updateWithLoopClosure(const Transformation &T_M_B_before,
                             const Transformation &T_M_B_after);

  void registerPointcloud(const sensor_msgs::PointCloud2::Ptr &pointcloud_msg);

  void publishTFs();
  void publishOdometry();

  // Transform getter methods
  Transformation get_T_O_B() { return T_O_B_; }
  Transformation get_T_M_B() { return T_M_L_ * T_L_O_ * T_O_B_; }
  Transformation get_T_M_C() { return T_M_L_ * T_L_O_ * T_O_B_ * T_B_C_; }

  const FrameNames &getFrameNames() const { return frame_names_; }

  // Config get/setters
  void setVerbosity(bool verbose) { verbose_ = verbose; }
  //  bool &useOdomFromTFs() {
  //    return use_odom_from_tfs_; }
  //  bool &useSensorCalibrationFromTFs() {
  //    return use_sensor_calibration_from_tfs_; }

 private:
  bool verbose_;

  ros::Time current_timestamp_;

  // Coordinate frame names
  // NOTE: This class is used to translate frame names between voxgraph
  //       and the other ROS nodes
  FrameNames frame_names_;

  // Transforms used to aggregate the incremental corrections
  Transformation T_M_L_;
  Transformation T_L_O_;

  // Transform that tracks the current odometry
  Transformation T_O_B_;

  // Transform from the pointcloud sensor frame to the robot's base_link
  Transformation T_B_C_;

  // Transformer class to lookup transforms from the TF tree or an odom topic
  bool use_odom_from_tfs_ = true;
  // NOTE: use_odom_tfs_ is automatically set to true if subscribeToTopics()
  //       is called with a non-empty odometry_input_topic argument
  bool use_sensor_calibration_from_tfs_ = true;
  // TODO(victorr): use_sensor_calibration_from_tfs_ is automatically set to
  //                false if a valid calibration is provided through ROS params
  TfTransformer tf_transformer_;
  OdometryTransformer odom_transformer_;

  // Scan to submap registerer used to refine the odometry estimate,
  // akin to voxblox ICP
  ScanToMapRegisterer scan_to_map_registerer_;

  // Odometry input
  ros::Subscriber imu_biases_subscriber_;
  BiasVectorType forwarded_accel_bias_ = BiasVectorType::Zero();
  BiasVectorType forwarded_gyro_bias_ = BiasVectorType::Zero();

  // Odometry output
  ros::Publisher odom_with_imu_biases_pub_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_FRONTEND_MAP_TRACKER_MAP_TRACKER_H_
