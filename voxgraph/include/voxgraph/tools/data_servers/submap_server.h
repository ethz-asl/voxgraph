#ifndef VOXGRAPH_TOOLS_DATA_SERVERS_SUBMAP_SERVER_H_
#define VOXGRAPH_TOOLS_DATA_SERVERS_SUBMAP_SERVER_H_

#include <std_msgs/Header.h>
#include <voxgraph_msgs/MapHeader.h>
#include "voxgraph/common.h"
#include "voxgraph/frontend/submap_collection/voxgraph_submap.h"

namespace voxgraph {
class SubmapServer {
 public:
  explicit SubmapServer(ros::NodeHandle nh_private);

  // Publish maps using the publishers that are members of this server instance
  void publishSubmap(const VoxgraphSubmap& submap, const ros::Time& timestamp);
  void publishSubmapTsdf(const VoxgraphSubmap& submap,
                         const ros::Time& timestamp);
  void publishSubmapEsdf(const VoxgraphSubmap& submap,
                         const ros::Time& timestamp);
  void publishSubmapSurfacePointcloud(const VoxgraphSubmap& submap,
                                      const ros::Time& timestamp);

  // "Bring your own publisher" methods
  // NOTE: These methods are provided s.t. they can be called using publishers
  //       to custom topics and without requiring a SubmapServer instance.
  //       They are therefore static.
  static void publishSubmapTsdf(const VoxgraphSubmap& submap,
                                const ros::Time& timestamp,
                                const ros::Publisher& submap_tsdf_publisher);
  static void publishSubmapEsdf(const VoxgraphSubmap& submap,
                                const ros::Time& timestamp,
                                const ros::Publisher& submap_esdf_publisher);
  static void publishSubmapSurfacePointcloud(
      const VoxgraphSubmap& submap, const ros::Time& timestamp,
      const ros::Publisher& submap_surface_pointcloud_publisher);

 private:
  ros::Publisher submap_tsdf_pub_;
  ros::Publisher submap_esdf_pub_;
  ros::Publisher submap_surface_pointcloud_pub_;

  static constexpr bool fake_6dof_transforms_ = true;

  // Convenience methods to generate the message and submap headers
  static std_msgs::Header generateHeaderMsg(const VoxgraphSubmap& submap,
                                            const ros::Time& timestamp);
  static voxgraph_msgs::MapHeader generateSubmapHeaderMsg(
      const VoxgraphSubmap& submap);

  // Conversion method from Kindr transforms to Eigen Affine3f transforms
  static void transformKindrToEigen(const Transformation& kindr,
                                    Eigen::Affine3f* eigen) {
    CHECK_NOTNULL(eigen);
    *eigen =
        Eigen::Translation3f(kindr.getPosition()) * kindr.getEigenQuaternion();
  }
};
}  // namespace voxgraph

#endif  // VOXGRAPH_TOOLS_DATA_SERVERS_SUBMAP_SERVER_H_
