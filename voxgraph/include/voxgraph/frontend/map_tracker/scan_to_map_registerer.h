#ifndef VOXGRAPH_FRONTEND_MAP_TRACKER_SCAN_TO_MAP_REGISTERER_H_
#define VOXGRAPH_FRONTEND_MAP_TRACKER_SCAN_TO_MAP_REGISTERER_H_

#include <ceres/ceres.h>
#include <sensor_msgs/PointCloud2.h>
#include <utility>
#include <Eigen/Core>
#include "voxgraph/backend/node/pose/pose_6d.h"
#include "voxgraph/frontend/map_tracker/cost_functions/scan_registration_cost_function.h"
#include "voxgraph/frontend/submap_collection/voxgraph_submap_collection.h"

namespace voxgraph {
class ScanToMapRegisterer {
 public:
  explicit ScanToMapRegisterer(
      VoxgraphSubmapCollection::ConstPtr submap_collection_ptr,
      bool verbose = false)
      : submap_collection_ptr_(std::move(submap_collection_ptr)),
        verbose_(verbose) {}

  bool refineSensorPose(const sensor_msgs::PointCloud2::Ptr &pointcloud_msg,
                        const Transformation &T_world__sensor_prior,
                        Transformation *T_world__sensor_refined) const;

  void setVerbosity(bool verbose) { verbose_ = verbose; }

 private:
  bool verbose_;
  VoxgraphSubmapCollection::ConstPtr submap_collection_ptr_;

  // Define the logging format used for Eigen matrices
  Eigen::IOFormat ioformat_{
    4, Eigen::DontAlignCols, "; ", "; ", "", "", "", ""};
};
}  // namespace voxgraph

#endif  // VOXGRAPH_FRONTEND_MAP_TRACKER_SCAN_TO_MAP_REGISTERER_H_
