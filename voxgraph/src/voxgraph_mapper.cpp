//
// Created by victor on 16.11.18.
//

#include "voxgraph/voxgraph_mapper.h"
#include <visualization_msgs/MarkerArray.h>
#include <voxblox_ros/ros_params.h>
#include "voxgraph/submap_registration/submap_registerer.h"

namespace voxgraph {
VoxgraphMapper::VoxgraphMapper(const ros::NodeHandle &nh,
                               const ros::NodeHandle &nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      transformer_(nh, nh_private),
      subscriber_queue_length_(100),
      tsdf_submap_collection_(tsdf_map_config_),
      tsdf_submap_mesher_(tsdf_map_config_, mesh_integrator_config_),
      odom_simulator_noise_linear_vel_dist_(
          odom_simulator_noise_linear_vel_mean_,
          odom_simulator_noise_linear_vel_stddev_),
      odom_simulator_noise_angular_vel_dist_(
          odom_simulator_noise_angular_vel_mean_,
          odom_simulator_noise_angular_vel_stddev_) {
  // Setup interaction with ROS
  getParametersFromRos();
  subscribeToTopics();
  advertiseTopics();
  advertiseServices();
}

void VoxgraphMapper::getParametersFromRos() {
  nh_private_.param("subscriber_queue_length", subscriber_queue_length_,
                    subscriber_queue_length_);
  nh_private_.param("verbose", verbose_, verbose_);
  nh_private_.param("use_tf_to_get_pointcloud_poses",
                    use_tf_to_get_pointcloud_poses_,
                    use_tf_to_get_pointcloud_poses_);

  double submap_creation_interval_temp;
  if (nh_private_.getParam("submap_creation_interval",
                           submap_creation_interval_temp)) {
    submap_creation_interval_ = ros::Duration(submap_creation_interval_temp);
  }

  // Read odometry simulator params from ROS
  nh_private_.param("odom_base_frame", odom_base_frame_, odom_base_frame_);
  nh_private_.param("odom_simulator_noise_linear_vel_mean",
                    odom_simulator_noise_linear_vel_mean_,
                    odom_simulator_noise_linear_vel_mean_);
  nh_private_.param("odom_simulator_noise_linear_vel_stddev",
                    odom_simulator_noise_linear_vel_stddev_,
                    odom_simulator_noise_linear_vel_stddev_);
  odom_simulator_noise_linear_vel_dist_ =
      std::normal_distribution<double>(odom_simulator_noise_linear_vel_mean_,
                                       odom_simulator_noise_linear_vel_stddev_);
  nh_private_.param("odom_simulator_noise_angular_vel_mean",
                    odom_simulator_noise_angular_vel_mean_,
                    odom_simulator_noise_angular_vel_mean_);
  nh_private_.param("odom_simulator_noise_angular_vel_stddev",
                    odom_simulator_noise_angular_vel_stddev_,
                    odom_simulator_noise_angular_vel_stddev_);
  odom_simulator_noise_angular_vel_dist_ = std::normal_distribution<double>(
      odom_simulator_noise_angular_vel_mean_,
      odom_simulator_noise_angular_vel_stddev_);

  // Read TSDF integrator parameters from ROS
  tsdf_integrator_config_ =
      voxblox::getTsdfIntegratorConfigFromRosParam(nh_private_);
}

void VoxgraphMapper::subscribeToTopics() {
  odometry_subscriber_ = nh_.subscribe("odometry", subscriber_queue_length_,
                                       &VoxgraphMapper::odometryCallback, this);
  absolute_pose_subscriber_ =
      nh_.subscribe("pose", subscriber_queue_length_,
                    &VoxgraphMapper::absolutePoseCallback, this);
  pointcloud_subscriber_ =
      nh_.subscribe("pointcloud", subscriber_queue_length_,
                    &VoxgraphMapper::pointcloudCallback, this);
}

void VoxgraphMapper::advertiseTopics() {
  separated_mesh_pub_ = nh_private_.advertise<visualization_msgs::Marker>(
      "separated_mesh", subscriber_queue_length_);
  combined_mesh_pub_ = nh_private_.advertise<visualization_msgs::Marker>(
      "combined_mesh", subscriber_queue_length_);
}

void VoxgraphMapper::advertiseServices() {
  publish_separated_mesh_srv_ = nh_private_.advertiseService(
      "publish_separated_mesh", &VoxgraphMapper::publishSeparatedMeshCallback,
      this);
  publish_combined_mesh_srv_ = nh_private_.advertiseService(
      "publish_combined_mesh", &VoxgraphMapper::publishCombinedMeshCallback,
      this);
  optimize_graph_srv_ = nh_private_.advertiseService(
      "optimize_graph", &VoxgraphMapper::optimizeGraphCallback, this);
  save_to_file_srv_ = nh_private_.advertiseService(
      "save_to_file", &VoxgraphMapper::saveToFileCallback, this);
}

void VoxgraphMapper::odometryCallback(
    const nav_msgs::Odometry::ConstPtr &odometry_msg) {
  // If this is the first odometry msg,
  // we do not know Dt yet and postpone integration to the next msg
  if (odom_simulator_pose_stamped_.header.stamp.isZero()) {
    ROS_INFO("Initialized drifting odometry simulator");
    odom_simulator_pose_stamped_.header.stamp = odometry_msg->header.stamp;
    // Initialize pose (note: important that orientation is correct)
    odom_simulator_pose_stamped_.pose = odometry_msg->pose.pose;
    return;
  }
  // Calculate time delta since last message
  double Dt =
      (odometry_msg->header.stamp - odom_simulator_pose_stamped_.header.stamp)
          .toSec();

  // Load quantities of interest
  Eigen::Vector3d position;  // Previous pose position in world frame
  tf::pointMsgToEigen(odom_simulator_pose_stamped_.pose.position, position);
  Eigen::Quaterniond orientation;  // Previous pose orientation in world frame
  tf::quaternionMsgToEigen(odom_simulator_pose_stamped_.pose.orientation,
                           orientation);
  Eigen::Vector3d linear_velocity__bodyframe;
  tf::vectorMsgToEigen(odometry_msg->twist.twist.linear,
                       linear_velocity__bodyframe);
  Eigen::Vector3d angular_velocity__bodyframe;
  tf::vectorMsgToEigen(odometry_msg->twist.twist.angular,
                       angular_velocity__bodyframe);

  // Simulate noise on measured velocities
  // TODO(victorr): Sample the noise from a multivariate gaussian
  //  (instead of assuming x,y,z are uncorrelated)
  linear_velocity__bodyframe.x() +=
      odom_simulator_noise_linear_vel_dist_(odom_simulator_noise_generator_);
  linear_velocity__bodyframe.y() +=
      odom_simulator_noise_linear_vel_dist_(odom_simulator_noise_generator_);
  linear_velocity__bodyframe.z() +=
      odom_simulator_noise_linear_vel_dist_(odom_simulator_noise_generator_);
  angular_velocity__bodyframe.x() +=
      odom_simulator_noise_angular_vel_dist_(odom_simulator_noise_generator_);
  angular_velocity__bodyframe.y() +=
      odom_simulator_noise_angular_vel_dist_(odom_simulator_noise_generator_);
  angular_velocity__bodyframe.z() +=
      odom_simulator_noise_angular_vel_dist_(odom_simulator_noise_generator_);

  // Express body frame angular and linear velocities in world frame
  Eigen::Vector3d linear_velocity__inertialframe =
      orientation * linear_velocity__bodyframe;
  Eigen::Vector3d angular_velocity__inertialframe =
      orientation * angular_velocity__bodyframe;

  // Integrate translation
  position.x() += linear_velocity__inertialframe.x() * Dt;
  position.y() += linear_velocity__inertialframe.y() * Dt;
  position.z() += linear_velocity__inertialframe.z() * Dt;

  // Integrate angular velocity
  // (angular velocity --integrate--> angle axis --> quaternion)
  Eigen::Quaterniond orientation_delta;
  Eigen::Vector3d half_angle_vector =
      angular_velocity__inertialframe * Dt * 0.5;  // Half angle vector
  double half_angle = half_angle_vector.norm();    // Half angle
  if (half_angle > 0.001) {
    // This accurate update is only applied
    // if the angular velocity is significant (unstable otherwise)
    double term = std::sin(half_angle) / half_angle;
    // --> Term s.t. half_angle_vector * term = axis * sin(angle/2)
    orientation_delta.w() = std::cos(half_angle);  // cos(angle/2)
    orientation_delta.x() =
        half_angle_vector.x() * term;  // axis.x * sin(angle/2)
    orientation_delta.y() =
        half_angle_vector.y() * term;  // axis.y * sin(angle/2)
    orientation_delta.z() =
        half_angle_vector.z() * term;  // axis.z * sin(angle/2)
  } else {
    // This update is a first order approximation around zero
    // (avoids division by 0 when angle -> 0)
    orientation_delta.w() = 1;
    orientation_delta.x() = half_angle_vector.x();
    orientation_delta.y() = half_angle_vector.y();
    orientation_delta.z() = half_angle_vector.z();
  }
  orientation = orientation_delta * orientation;
  orientation.normalize();

  // Update stored pose
  odom_simulator_pose_stamped_.header = odometry_msg->header;
  tf::quaternionEigenToMsg(orientation,
                           odom_simulator_pose_stamped_.pose.orientation);
  tf::pointEigenToMsg(position, odom_simulator_pose_stamped_.pose.position);
  /* TODO(victorr): Broadcast the noisy odom as a TF
   *                instead of storing it as a variable,
   *                this way time history will be available
   *                -> important for pointcloudCallback() */
}

void VoxgraphMapper::absolutePoseCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg) {}

void VoxgraphMapper::pointcloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr &pointcloud_msg) {
  // Get transformation corresponding to current odometry
  Eigen::Vector3d position;
  tf::pointMsgToEigen(odom_simulator_pose_stamped_.pose.position, position);
  Eigen::Quaterniond orientation;
  tf::quaternionMsgToEigen(odom_simulator_pose_stamped_.pose.orientation,
                           orientation);
  voxblox::Transformation T_world__odom(
      position.cast<voxblox::FloatingPoint>(),
      orientation.cast<voxblox::FloatingPoint>());

  // Check if it's time to create a new submap
  if (pointcloud_msg->header.stamp >
      current_submap_creation_stamp_ + submap_creation_interval_) {
    // Create new submap
    ROS_INFO_STREAM("Creating submap nr: " << tsdf_submap_collection_.size());
    std::cout << "with pose\n" << T_world__odom << std::endl;
    tsdf_submap_collection_.createNewSubMap(T_world__odom);
    current_submap_creation_stamp_ = pointcloud_msg->header.stamp;

    // Tell the TSDF integrator to update the new submap
    // TODO(victorr): Switch to cblox tsdf integrator
    if (tsdf_integrator_ == nullptr) {
      ROS_WARN("Created tsdf_integrator_ (was nullptr)");
      tsdf_integrator_.reset(new voxblox::FastTsdfIntegrator(
          tsdf_integrator_config_,
          tsdf_submap_collection_.getActiveTsdfMapPtr()->getTsdfLayerPtr()));
    } else {
      tsdf_integrator_->setLayer(
          tsdf_submap_collection_.getActiveTsdfMapPtr()->getTsdfLayerPtr());
    }

    // Show new submap coordinate frame in Rviz
    Eigen::Affine3d arrow_pose;
    arrow_pose = T_world__odom.getTransformationMatrix().cast<double_t>();
    std::string axis_label =
        std::to_string(tsdf_submap_collection_.getActiveSubMapID());
    //    visual_tools_->publishAxisLabeled(
    //        arrow_pose, axis_label, rviz_visual_tools::XLARGE);
    //    visual_tools_->trigger(); // To trigger the publisher

    // Show TSDF submap collection mesh in Rviz
    publishMesh(SEPARATED_MESH);
  }

  // Find transformation from pointcloud_msg's sensor to the world frame
  voxblox::Transformation T_world__sensor;
  if (use_tf_to_get_pointcloud_poses_) {
    transformer_.lookupTransform(pointcloud_msg->header.frame_id, "world",
                                 pointcloud_msg->header.stamp,
                                 &T_world__sensor);
  } else {
    // Transformation from odometry frame to pointcloud_msg sensor
    voxblox::Transformation T_odom__sensor;
    transformer_.lookupTransform(pointcloud_msg->header.frame_id,
                                 odom_base_frame_, pointcloud_msg->header.stamp,
                                 &T_odom__sensor);
    T_world__sensor = T_world__odom * T_odom__sensor;
  }
  // Find relative transform from
  // pointcloud_msg sensor to the current submap frame
  const cblox::Transformation T_world__submap =
      tsdf_submap_collection_.getActiveSubMapPose();
  const cblox::Transformation T_submap__sensor =
      T_world__submap.inverse() * T_world__sensor;

  // Visualize transformations for debugging
  Eigen::Affine3d TF_world__submap(
      T_world__submap.getTransformationMatrix().cast<double_t>());
  //  tf_visualizer_.publishTransform(
  //      TF_world__submap, "world", "T_world_submap");
  Eigen::Affine3d TF_world__odom(
      T_world__odom.getTransformationMatrix().cast<double_t>());
  //  tf_visualizer_.publishTransform(TF_world__odom, "world", "T_world__odom");
  Eigen::Affine3d TF_world__sensor(
      T_world__sensor.getTransformationMatrix().cast<double_t>());
  //  tf_visualizer_.publishTransform(
  //      TF_world__sensor, "world", "T_world__sensor");

  // Convert pointcloud_msg into voxblox::Pointcloud
  pcl::PointCloud<pcl::PointXYZRGB> pointcloud_pcl;
  pcl::fromROSMsg(*pointcloud_msg, pointcloud_pcl);
  voxblox::Pointcloud points_C;
  voxblox::Colors colors;
  points_C.reserve(pointcloud_pcl.size());
  colors.reserve(pointcloud_pcl.size());

  // Filter out NaNs while compiling the voxblox cloud
  for (size_t i = 0; i < pointcloud_pcl.points.size(); ++i) {
    if (!std::isfinite(pointcloud_pcl.points[i].x) ||
        !std::isfinite(pointcloud_pcl.points[i].y) ||
        !std::isfinite(pointcloud_pcl.points[i].z)) {
      continue;
    }

    points_C.push_back(voxblox::Point(pointcloud_pcl.points[i].x,
                                      pointcloud_pcl.points[i].y,
                                      pointcloud_pcl.points[i].z));
    colors.push_back(
        voxblox::Color(pointcloud_pcl.points[i].r, pointcloud_pcl.points[i].g,
                       pointcloud_pcl.points[i].b, pointcloud_pcl.points[i].a));
  }

  // Double check if the tsdf_integrator_ is initialized
  CHECK_NOTNULL(tsdf_integrator_);

  // Integrate the pointcloud (and report timings if requested)
  ROS_INFO_COND(verbose_, "Integrating a pointcloud with %lu points.",
                points_C.size());
  ros::WallTime start = ros::WallTime::now();
  tsdf_integrator_->integratePointCloud(T_submap__sensor, points_C, colors);
  ros::WallTime end = ros::WallTime::now();
  ROS_INFO_COND(
      verbose_,
      "Finished integrating in %f seconds, submap %u now has %lu blocks.",
      (end - start).toSec(), tsdf_submap_collection_.getActiveSubMapID(),
      tsdf_submap_collection_.getActiveTsdfMap()
          .getTsdfLayer()
          .getNumberOfAllocatedBlocks());
}

void VoxgraphMapper::publishMesh(const voxblox::MeshLayer::Ptr &mesh_layer_ptr,
                                 const ros::Publisher &publisher,
                                 const voxblox::ColorMode &color_mode,
                                 const std::string &reference_frame) {
  // Publish mesh for visualization
  visualization_msgs::Marker marker;
  voxblox::fillMarkerWithMesh(mesh_layer_ptr, color_mode, &marker);
  marker.header.frame_id = reference_frame;
  // Retransform the marker according to its TF frame at each time step
  marker.frame_locked = true;
  publisher.publish(marker);
}

void VoxgraphMapper::publishMesh(MeshType mesh_type) {
  cblox::MeshLayer::Ptr mesh_layer_ptr(
      new cblox::MeshLayer(tsdf_submap_collection_.block_size()));
  switch (mesh_type) {
    case COMBINED_MESH:
      generateMesh(COMBINED_MESH, mesh_layer_ptr);
      publishMesh(mesh_layer_ptr, combined_mesh_pub_,
                  voxblox::ColorMode::kNormals);  // Publish colored by Normals
      break;
    case SEPARATED_MESH:
      generateMesh(SEPARATED_MESH, mesh_layer_ptr);
      publishMesh(mesh_layer_ptr, separated_mesh_pub_,
                  voxblox::ColorMode::kColor);  // Publish colored by submap ID
      break;
  }
}

void VoxgraphMapper::generateMesh(
    const MeshType &mesh_type, const voxblox::MeshLayer::Ptr &mesh_layer_ptr) {
  // Generate the separated mesh into a new layer
  switch (mesh_type) {
    case SEPARATED_MESH:
      ROS_INFO_STREAM_COND(verbose_, "Generating Separated mesh");
      tsdf_submap_mesher_.generateSeparatedMesh(tsdf_submap_collection_,
                                                mesh_layer_ptr.get());
      break;
    case COMBINED_MESH:
      ROS_INFO_STREAM_COND(verbose_, "Generating Combined mesh");
      tsdf_submap_mesher_.generateCombinedMesh(tsdf_submap_collection_,
                                               mesh_layer_ptr.get());
      break;
  }
}

bool VoxgraphMapper::publishSeparatedMeshCallback(
    std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {
  publishMesh(SEPARATED_MESH);
  return true;  // Tell ROS it succeeded
}

bool VoxgraphMapper::publishCombinedMeshCallback(
    std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {
  publishMesh(COMBINED_MESH);
  return true;  // Tell ROS it succeeded
}

bool VoxgraphMapper::optimizeGraphCallback(
    std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {
  ROS_WARN("Graph optimization is not yet ready.");
  return true;
}

bool VoxgraphMapper::saveToFileCallback(
    voxblox_msgs::FilePath::Request &request,
    voxblox_msgs::FilePath::Response &response) {
  tsdf_submap_collection_.saveToFile(request.file_path);
  return true;
}
}  // namespace voxgraph
