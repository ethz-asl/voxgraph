//
// Created by victor on 16.11.18.
//

#include "voxgraph/frontend/voxgraph_mapper.h"
#include <minkindr_conversions/kindr_xml.h>
#include <visualization_msgs/MarkerArray.h>
#include <memory>
#include <string>
#include <thread>
#include "voxgraph/backend/constraint/cost_functions/submap_registration/submap_registerer.h"
#include "voxgraph/frontend/submap_collection/submap_timeline.h"
#include "voxgraph/tools/tf_helper.h"

namespace voxgraph {
VoxgraphMapper::VoxgraphMapper(const ros::NodeHandle &nh,
                               const ros::NodeHandle &nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      verbose_(false),
      debug_(false),
      subscriber_queue_length_(100),
      pointcloud_topic_("pointcloud"),
      transformer_(nh, nh_private),
      world_frame_("world"),
      odom_frame_("odom"),
      robot_frame_("robot"),
      use_gt_ptcloud_pose_from_sensor_tf_(false),
      submap_collection_ptr_(
          std::make_shared<VoxgraphSubmapCollection>(submap_config_)),
      pose_graph_interface_(nh_private, submap_collection_ptr_),
      pointcloud_processor_(submap_collection_ptr_),
      submap_vis_(submap_config_),
      registration_constraints_enabled_(false),
      odometry_constraints_enabled_(false),
      height_constraints_enabled_(false),
      rosbag_helper_(nh) {
  // Setup interaction with ROS
  getParametersFromRos();
  subscribeToTopics();
  advertiseTopics();
  advertiseServices();

  // Publish the initial transform from the world to the drifting odom frame
  TfHelper::publishTransform(voxblox::Transformation(), world_frame_,
                             odom_frame_corrected_, true);
}

void VoxgraphMapper::getParametersFromRos() {
  nh_private_.param("debug", debug_, debug_);
  nh_private_.param("verbose", verbose_, verbose_);
  pose_graph_interface_.setVerbosity(verbose_);

  nh_private_.param("subscriber_queue_length", subscriber_queue_length_,
                    subscriber_queue_length_);
  nh_private_.param("pointcloud_topic", pointcloud_topic_, pointcloud_topic_);
  nh_private_.param("world_frame", world_frame_, world_frame_);
  nh_private_.param("odom_frame", odom_frame_, odom_frame_);
  nh_private_.param("robot_frame", robot_frame_, robot_frame_);
  nh_private_.param("odom_frame_corrected", odom_frame_corrected_,
                    odom_frame_corrected_);
  nh_private_.param("robot_frame_corrected", robot_frame_corrected_,
                    robot_frame_corrected_);
  nh_private_.param("sensor_frame_corrected", sensor_frame_corrected_,
                    sensor_frame_corrected_);
  nh_private_.param("use_gt_ptcloud_pose_from_sensor_tf",
                    use_gt_ptcloud_pose_from_sensor_tf_,
                    use_gt_ptcloud_pose_from_sensor_tf_);

  // Get the submap creation interval as a ros::Duration
  double interval_temp;
  if (nh_private_.getParam("submap_creation_interval", interval_temp)) {
    submap_collection_ptr_->setSubmapCreationInterval(
        ros::Duration(interval_temp));
  }

  // Load the transform from the robot frame to the sensor frame
  XmlRpc::XmlRpcValue T_robot_sensor_xml;
  CHECK(nh_private_.getParam("T_robot_sensor", T_robot_sensor_xml))
      << "The transform from the robot frame to the sensor frame "
      << "T_robot_sensor is required but was not set.";
  kindr::minimal::xmlRpcToKindr(T_robot_sensor_xml, &T_robot_sensor_);

  // Read the measurement params from their sub-namespace
  ros::NodeHandle nh_measurement_params(nh_private_, "measurements");
  nh_measurement_params.param("submap_registration/enabled",
                              registration_constraints_enabled_,
                              registration_constraints_enabled_);
  ROS_INFO_STREAM_COND(
      verbose_,
      "Submap registration constraints: "
          << (registration_constraints_enabled_ ? "enabled" : "disabled"));
  nh_measurement_params.param("odometry/enabled", odometry_constraints_enabled_,
                              odometry_constraints_enabled_);
  ROS_INFO_STREAM_COND(
      verbose_,
      "Odometry constraints: " << (odometry_constraints_enabled_ ? "enabled"
                                                                 : "disabled"));
  nh_measurement_params.param("height/enabled", height_constraints_enabled_,
                              height_constraints_enabled_);
  ROS_INFO_STREAM_COND(
      verbose_, "Height constraints: "
                    << (height_constraints_enabled_ ? "enabled" : "disabled"));
  pose_graph_interface_.setPoseGraphConfigFromRosParams(nh_measurement_params);

  // Read TSDF integrator params from their sub-namespace
  ros::NodeHandle nh_tsdf_params(nh_private_, "tsdf_integrator");
  pointcloud_processor_.setTsdfIntegratorConfigFromRosParam(nh_tsdf_params);
}

void VoxgraphMapper::subscribeToTopics() {
  pointcloud_subscriber_ =
      nh_.subscribe(pointcloud_topic_, subscriber_queue_length_,
                    &VoxgraphMapper::pointcloudCallback, this);
}

void VoxgraphMapper::advertiseTopics() {
  separated_mesh_pub_ = nh_private_.advertise<visualization_msgs::Marker>(
      "separated_mesh", subscriber_queue_length_, true);
  combined_mesh_pub_ = nh_private_.advertise<visualization_msgs::Marker>(
      "combined_mesh", subscriber_queue_length_, true);
}

void VoxgraphMapper::advertiseServices() {
  publish_separated_mesh_srv_ = nh_private_.advertiseService(
      "publish_separated_mesh", &VoxgraphMapper::publishSeparatedMeshCallback,
      this);
  publish_combined_mesh_srv_ = nh_private_.advertiseService(
      "publish_combined_mesh", &VoxgraphMapper::publishCombinedMeshCallback,
      this);
  save_to_file_srv_ = nh_private_.advertiseService(
      "save_to_file", &VoxgraphMapper::saveToFileCallback, this);
}

bool VoxgraphMapper::publishSeparatedMeshCallback(
    std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {
  submap_vis_.publishSeparatedMesh(*submap_collection_ptr_, world_frame_,
                                   separated_mesh_pub_);
  return true;  // Tell ROS it succeeded
}

bool VoxgraphMapper::publishCombinedMeshCallback(
    std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {
  submap_vis_.publishCombinedMesh(*submap_collection_ptr_, world_frame_,
                                  combined_mesh_pub_);
  return true;  // Tell ROS it succeeded
}

bool VoxgraphMapper::saveToFileCallback(
    voxblox_msgs::FilePath::Request &request,
    voxblox_msgs::FilePath::Response &response) {
  submap_collection_ptr_->saveToFile(request.file_path);
  return true;
}

void VoxgraphMapper::pointcloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr &pointcloud_msg) {
  // Lookup the robot pose at the time of the pointcloud message
  ros::Time current_timestamp = pointcloud_msg->header.stamp;
  Transformation T_odom_robot;
  if (!lookup_T_odom_robot(current_timestamp, &T_odom_robot)) {
    // If the pose cannot be found, the pointcloud is skipped
    ROS_WARN_STREAM("Skipping pointcloud since the robot pose at time "
                    << current_timestamp << " is unknown.");
    return;
  }
  Transformation T_world_robot = T_world_odom_corrected_ * T_odom_robot;

  // Check if it's time to create a new submap
  if (submap_collection_ptr_->shouldCreateNewSubmap(current_timestamp)) {
    // Add the finished submap to the pose graph, optimize and correct for drift
    // NOTE: We only add submaps to the pose graph once they're finished to
    //       avoid generating their ESDF more than once
    if (!submap_collection_ptr_->empty()) {
      // Pause the rosbag (remove this once the system runs in real-time)
      // TODO(victorr): Parametrize this
      if (false) {
        rosbag_helper_.pauseRosbag();
      }

      // Add the finished submap to the pose graph, including an odometry link
      SubmapID finished_submap_id = submap_collection_ptr_->getActiveSubMapID();
      pose_graph_interface_.addSubmap(finished_submap_id,
                                      odometry_constraints_enabled_);

      // Constrain the height
      // NOTE: No constraint is added for the 1st since its pose is fixed
      if (height_constraints_enabled_ && submap_collection_ptr_->size() > 1) {
        // This assumes that the height estimate from the odometry source is
        // really good (e.g. when it fuses an altimeter)
        double current_height = T_odom_robot.getPosition().z();
        pose_graph_interface_.addHeightMeasurement(finished_submap_id,
                                                   current_height);
      }

      // Optimize the pose graph
      ROS_INFO("Optimizing the pose graph");
      if (registration_constraints_enabled_) {
        pose_graph_interface_.updateRegistrationConstraints();
      }
      pose_graph_interface_.optimize();

      // Remember the pose of the current submap (used later to eliminate drift)
      Transformation T_W_S_old = submap_collection_ptr_->getActiveSubMapPose();

      // Update the submap poses
      pose_graph_interface_.updateSubmapCollectionPoses();

      // Update the fictional odometry origin to cancel out drift
      Transformation T_W_S_new = submap_collection_ptr_->getActiveSubMapPose();
      T_world_odom_corrected_ =
          T_W_S_new * T_W_S_old.inverse() * T_world_odom_corrected_,
      T_world_robot = T_W_S_new * T_W_S_old.inverse() * T_world_robot;
      ROS_INFO_STREAM("Applying pose correction:\n"
                      << T_W_S_new * T_W_S_old.inverse());

      // Publish the new corrected odom frame
      TfHelper::publishTransform(T_world_odom_corrected_, world_frame_,
                                 odom_frame_corrected_, true,
                                 current_timestamp);

      // Update the submap collection visualization in Rviz
      std::thread meshing_thread(&SubmapVisuals::publishCombinedMesh,
                                 &submap_vis_, *submap_collection_ptr_,
                                 world_frame_, combined_mesh_pub_);
      meshing_thread.detach();

      // Play the rosbag (remove this once the system runs in real-time)
      // TODO(victorr): Parametrize this
      if (false) {
        rosbag_helper_.playRosbag();
      }
    }

    // Create the new submap
    submap_collection_ptr_->createNewSubmap(T_world_robot, current_timestamp);
    if (debug_) {
      TfHelper::publishTransform(submap_collection_ptr_->getActiveSubMapPose(),
                                 world_frame_, "new_submap_origin", true,
                                 current_timestamp);
    }
  }

  // Lookup the sensor's pose in world frame
  Transformation T_world_sensor;
  if (use_gt_ptcloud_pose_from_sensor_tf_) {
    ROS_WARN_STREAM_THROTTLE(20, "Using ground truth pointcloud poses"
                                     << " provided by TF from " << world_frame_
                                     << " to "
                                     << pointcloud_msg->header.frame_id);
    transformer_.lookupTransform(pointcloud_msg->header.frame_id, world_frame_,
                                 current_timestamp, &T_world_sensor);
    if (debug_) {
      TfHelper::publishTransform(T_world_sensor, world_frame_, "sensor", false,
                                 current_timestamp);
    }
  } else {
    // Get the pose of the sensor in world frame
    T_world_sensor = T_world_robot * T_robot_sensor_;
    if (debug_) {
      TfHelper::publishTransform(T_world_robot, world_frame_,
                                 robot_frame_corrected_, false,
                                 current_timestamp);
      TfHelper::publishTransform(T_robot_sensor_, robot_frame_corrected_,
                                 sensor_frame_corrected_, true,
                                 current_timestamp);
    }
  }

  // Integrate the pointcloud
  pointcloud_processor_.integratePointcloud(pointcloud_msg, T_world_sensor);

  // Add the current pose to the submap's pose history
  submap_collection_ptr_->getActiveSubMapPtr()->addPoseToHistory(
      current_timestamp, T_world_robot);
}

bool VoxgraphMapper::lookup_T_odom_robot(ros::Time timestamp,
                                         Transformation *T_odom_robot) {
  CHECK_NOTNULL(T_odom_robot);
  Transformation T_odom_robot_received;
  double t_waited = 0;   // Total time spent waiting for the updated pose
  double t_max = 0.20;   // Maximum time to wait before giving up
  const ros::Duration timeout(0.005);  // Timeout between each update attempt
  while (t_waited < t_max) {
    if (transformer_.lookupTransform(robot_frame_, odom_frame_, timestamp,
                                     &T_odom_robot_received)) {
      *T_odom_robot = T_odom_robot_received;
      return true;
    }
    timeout.sleep();
    t_waited += timeout.toSec();
  }
  ROS_WARN("Waited %.3fs, but still could not get the updated odom", t_waited);
  return false;
}
}  // namespace voxgraph
