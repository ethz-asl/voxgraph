//
// Created by victor on 16.11.18.
//

#include "voxgraph/mapper/voxgraph_mapper.h"
#include <visualization_msgs/MarkerArray.h>
#include <voxblox_ros/ros_params.h>
#include <string>
#include "voxgraph/submap_registration/submap_registerer.h"

namespace voxgraph {
VoxgraphMapper::VoxgraphMapper(const ros::NodeHandle &nh,
                               const ros::NodeHandle &nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      transformer_(nh, nh_private),
      odom_base_frame_("odom"),
      subscriber_queue_length_(100),
      submap_collection_(submap_config_),
      submap_vis_(submap_config_) {
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

  // Get the submap creation interval as a ros::Duration
  double submap_creation_interval_temp;
  if (nh_private_.getParam("submap_creation_interval",
                           submap_creation_interval_temp)) {
    submap_creation_interval_ = ros::Duration(submap_creation_interval_temp);
  }

  // Read TSDF integrator parameters from ROS
  tsdf_integrator_config_ =
      voxblox::getTsdfIntegratorConfigFromRosParam(nh_private_);
}

void VoxgraphMapper::subscribeToTopics() {
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
  save_to_file_srv_ = nh_private_.advertiseService(
      "save_to_file", &VoxgraphMapper::saveToFileCallback, this);
}

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
    ROS_INFO_STREAM("Creating submap nr: " << submap_collection_.size());
    voxblox::Transformation::Vector6 T_vec = T_world__odom.log();
    T_vec[3] = 0;
    T_vec[4] = 0;
    voxblox::Transformation T_world__new_submap =
        voxblox::Transformation::exp(T_vec);
    std::cout << "with pose\n" << T_world__new_submap << std::endl;
    submap_collection_.createNewSubMap(T_world__new_submap);
    current_submap_creation_stamp_ = pointcloud_msg->header.stamp;

    // Tell the TSDF integrator to update the new submap
    if (tsdf_integrator_ == nullptr) {
      ROS_WARN("Created tsdf_integrator_ (was nullptr)");
      tsdf_integrator_.reset(new voxblox::FastTsdfIntegrator(
          tsdf_integrator_config_,
          submap_collection_.getActiveTsdfMapPtr()->getTsdfLayerPtr()));
    } else {
      tsdf_integrator_->setLayer(
          submap_collection_.getActiveTsdfMapPtr()->getTsdfLayerPtr());
    }

    // Show new submap coordinate frame in Rviz
    Eigen::Affine3d arrow_pose;
    arrow_pose = T_world__odom.getTransformationMatrix().cast<double_t>();
    std::string axis_label =
        std::to_string(submap_collection_.getActiveSubMapID());
    //    visual_tools_->publishAxisLabeled(
    //        arrow_pose, axis_label, rviz_visual_tools::XLARGE);
    //    visual_tools_->trigger(); // To trigger the publisher

    // Show TSDF submap collection mesh in Rviz
    submap_vis_.publishSeparatedMesh(submap_collection_, "world",
                                     separated_mesh_pub_);
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
      submap_collection_.getActiveSubMapPose();
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
      (end - start).toSec(), submap_collection_.getActiveSubMapID(),
      submap_collection_.getActiveTsdfMap()
          .getTsdfLayer()
          .getNumberOfAllocatedBlocks());
}

bool VoxgraphMapper::publishSeparatedMeshCallback(
    std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {
  submap_vis_.publishSeparatedMesh(submap_collection_, "world",
                                   separated_mesh_pub_);
  return true;  // Tell ROS it succeeded
}

bool VoxgraphMapper::publishCombinedMeshCallback(
    std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {
  submap_vis_.publishCombinedMesh(submap_collection_, "world",
                                  combined_mesh_pub_);
  return true;  // Tell ROS it succeeded
}

bool VoxgraphMapper::saveToFileCallback(
    voxblox_msgs::FilePath::Request &request,
    voxblox_msgs::FilePath::Response &response) {
  submap_collection_.saveToFile(request.file_path);
  return true;
}
}  // namespace voxgraph
