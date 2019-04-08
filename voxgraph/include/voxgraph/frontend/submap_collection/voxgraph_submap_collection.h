//
// Created by victor on 07.04.19.
//

#ifndef VOXGRAPH_FRONTEND_SUBMAP_COLLECTION_VOXGRAPH_SUBMAP_COLLECTION_H_
#define VOXGRAPH_FRONTEND_SUBMAP_COLLECTION_VOXGRAPH_SUBMAP_COLLECTION_H_

#include <cblox/core/common.h>
#include <cblox/core/submap_collection.h>
#include <ros/ros.h>
#include <voxblox/core/common.h>
#include <memory>
#include <utility>
#include "voxgraph/common.h"
#include "voxgraph/frontend/submap_collection/voxgraph_submap.h"

namespace voxgraph {
class VoxgraphSubmapCollection
    : public cblox::SubmapCollection<VoxgraphSubmap> {
 public:
  typedef std::shared_ptr<VoxgraphSubmapCollection> Ptr;
  typedef std::shared_ptr<const VoxgraphSubmapCollection> ConstPtr;

  explicit VoxgraphSubmapCollection(VoxgraphSubmap::Config submap_config)
      : SubmapCollection(submap_config),
        submap_creation_interval_(20),  // In seconds
        current_submap_creation_stamp_(0) {}

  void setSubmapCreationInterval(ros::Duration submap_creation_interval) {
    submap_creation_interval_ = std::move(submap_creation_interval);
  }

  bool shouldCreateNewSubmap(const ros::Time &current_time);

  void createNewSubmap(const Transformation &T_world_robot,
                       const ros::Time &timestamp);

 private:
  // New submap creation stats
  ros::Duration submap_creation_interval_;
  ros::Time current_submap_creation_stamp_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_FRONTEND_SUBMAP_COLLECTION_VOXGRAPH_SUBMAP_COLLECTION_H_
