#ifndef VOXGRAPH_FRONTEND_MAP_TRACKER_SCAN_TO_MAP_REGISTERER_H_
#define VOXGRAPH_FRONTEND_MAP_TRACKER_SCAN_TO_MAP_REGISTERER_H_

#include <ceres/ceres.h>
#include <sensor_msgs/PointCloud2.h>
#include <utility>
#include "voxgraph/backend/node/pose/pose_6d.h"
#include "voxgraph/frontend/map_tracker/cost_functions/scan_registration_cost_function.h"
#include "voxgraph/frontend/submap_collection/voxgraph_submap_collection.h"

namespace voxgraph {
class ScanToMapRegisterer {
 public:
  explicit ScanToMapRegisterer(
      VoxgraphSubmapCollection::ConstPtr submap_collection_ptr)
      : submap_collection_ptr_(std::move(submap_collection_ptr)) {}

  bool refineOdometry(const sensor_msgs::PointCloud2::Ptr &pointcloud_msg,
                      const Transformation &T_submap__odometry_prior,
                      Transformation *T_submap__refined_odometry);

 private:
  VoxgraphSubmapCollection::ConstPtr submap_collection_ptr_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_FRONTEND_MAP_TRACKER_SCAN_TO_MAP_REGISTERER_H_
