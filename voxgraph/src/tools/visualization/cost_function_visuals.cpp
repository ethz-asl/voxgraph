#include "voxgraph/tools/visualization/cost_function_visuals.h"

namespace voxgraph {
CostFunctionVisuals::CostFunctionVisuals() {
  // Advertise the residual visualization pointcloud and Jacobian vector field
  ros::NodeHandle nh_private("~");
  residual_ptcloud_pub_ = nh_private.advertise<pcl::PointCloud<pcl::PointXYZI>>(
      "cost_residuals", 1, true);
  jacobians_pub_ = nh_private.advertise<visualization_msgs::MarkerArray>(
      "cost_jacobians", 1, true);

  // Configure the residual visualization pointcloud
  residual_ptcloud_.header.frame_id = "odom";

  // The magnitudes and orientations of the Jacobians
  // are visualized as a list of lines
  jacobian_arrows_.header.stamp = ros::Time::now();
  jacobian_arrows_.header.frame_id = "odom";
  jacobian_arrows_.ns = "jacobian_vectors";
  jacobian_arrows_.id = 1;
  jacobian_arrows_.type = visualization_msgs::Marker::LINE_LIST;
  jacobian_arrows_.action = visualization_msgs::Marker::ADD;
  jacobian_arrows_.pose.orientation.w = 1.0;  // Set to unit quaternion
  jacobian_arrows_.scale.x = 0.02;
  voxblox::colorVoxbloxToMsg(voxblox::Color::Red(), &jacobian_arrows_.color);
  jacobian_arrows_.frame_locked = false;

  // To show the direction of the Jacobians, their origins
  // are displayed as a list of sphere
  jacobian_origins_.header = jacobian_arrows_.header;
  jacobian_origins_.ns = "jacobian_origins";
  jacobian_origins_.id = 2;
  jacobian_origins_.type = visualization_msgs::Marker::SPHERE_LIST;
  jacobian_origins_.action = visualization_msgs::Marker::ADD;
  jacobian_origins_.pose.orientation.w = 1.0;  // Set to unit quaternion
  jacobian_origins_.scale.x = 0.05;
  jacobian_origins_.scale.y = 0.05;
  jacobian_origins_.scale.z = 0.05;
  voxblox::colorVoxbloxToMsg(voxblox::Color::Black(), &jacobian_origins_.color);
  jacobian_origins_.frame_locked = false;
}

void CostFunctionVisuals::addResidual(const voxblox::Point& coordinate,
                                      const double& residual) {
  pcl::PointXYZI point;
  point.x = coordinate.x();
  point.y = coordinate.y();
  point.z = coordinate.z();
  point.intensity = static_cast<float>(residual);
  residual_ptcloud_.push_back(point);
}

void CostFunctionVisuals::addJacobian(const voxblox::Point& coordinate,
                                      const voxblox::Point& jacobian) {
  // Add the current point to the Jacobian origin and arrow vectors
  geometry_msgs::Point coordinate_msg;
  tf::pointEigenToMsg(coordinate.cast<double>(), coordinate_msg);
  jacobian_origins_.points.push_back(coordinate_msg);
  jacobian_arrows_.points.push_back(coordinate_msg);
  // NOTE: The Jacobian is first stored in jacobian_arrows as is.
  // Once all Jacobians are known, they will be normalized and
  // converted into representative arrow end points. This happens
  // in the scaleAndPublish() method.
  geometry_msgs::Point jacobian_vector_msg;
  tf::pointEigenToMsg(jacobian.cast<double>(), jacobian_vector_msg);
  jacobian_arrows_.points.push_back(jacobian_vector_msg);
}

void CostFunctionVisuals::scaleAndPublish(const double factor) {
  size_t num_residuals = residual_ptcloud_.size();
  size_t num_jacobians = jacobian_origins_.points.size();
  // Scale the residuals
  if (num_residuals != 0) {
    for (size_t i = 0; i < num_residuals; i++) {
      residual_ptcloud_[i].intensity *= factor;
    }
  }
  // Scale the Jacobians
  if (num_jacobians != 0) {
    for (size_t i = 0; i < num_jacobians; i++) {
      // Scale the Jacobians for good visibility in Rviz
      jacobian_arrows_.points[1 + i * 2].x *= factor * 0.05;
      jacobian_arrows_.points[1 + i * 2].y *= factor * 0.05;
      jacobian_arrows_.points[1 + i * 2].z *= factor * 0.05;
      // Compute the arrow endpoint
      // by summing the arrow origin to the scaled Jacobian vector
      jacobian_arrows_.points[1 + i * 2].x += jacobian_origins_.points[i].x;
      jacobian_arrows_.points[1 + i * 2].y += jacobian_origins_.points[i].y;
      jacobian_arrows_.points[1 + i * 2].z += jacobian_origins_.points[i].z;
    }
  }
  // Publish the residuals and Jacobians, given that they are not empty
  if (num_residuals != 0) {
    residual_ptcloud_pub_.publish(residual_ptcloud_);
  }
  if (num_jacobians != 0) {
    jacobian_marker_array_.markers.emplace_back(jacobian_arrows_);
    jacobian_marker_array_.markers.emplace_back(jacobian_origins_);
    jacobians_pub_.publish(jacobian_marker_array_);
  }
}

void CostFunctionVisuals::reset() {
  // Reset the residual pointcloud
  residual_ptcloud_.clear();
  residual_ptcloud_.header.frame_id = "odom";
  // Reset variables used to visualize the Jacobians
  jacobian_marker_array_.markers.clear();
  jacobian_origins_.points.clear();
  jacobian_arrows_.points.clear();
}
}  // namespace voxgraph
