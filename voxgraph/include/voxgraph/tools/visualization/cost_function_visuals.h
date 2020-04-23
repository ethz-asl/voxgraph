#ifndef VOXGRAPH_TOOLS_VISUALIZATION_COST_FUNCTION_VISUALS_H_
#define VOXGRAPH_TOOLS_VISUALIZATION_COST_FUNCTION_VISUALS_H_

#include <voxblox/core/common.h>
#include <voxblox_ros/ptcloud_vis.h>

namespace voxgraph {
class CostFunctionVisuals {
 public:
  CostFunctionVisuals();

  void addResidual(const voxblox::Point& coordinate, const double& residual);
  void addJacobian(const voxblox::Point& coordinate,
                   const voxblox::Point& jacobian);

  void scaleAndPublish(const double factor);

  void reset();

 private:
  // ROS publishers
  ros::Publisher residual_ptcloud_pub_;
  ros::Publisher jacobians_pub_;

  // Pointcloud used to visualize the residuals
  pcl::PointCloud<pcl::PointXYZI> residual_ptcloud_;

  // Variables used to visualize the Jacobians,
  // see more details in the constructor
  visualization_msgs::MarkerArray jacobian_marker_array_;
  visualization_msgs::Marker jacobian_arrows_;
  visualization_msgs::Marker jacobian_origins_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_TOOLS_VISUALIZATION_COST_FUNCTION_VISUALS_H_
