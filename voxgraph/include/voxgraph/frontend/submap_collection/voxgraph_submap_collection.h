#ifndef VOXGRAPH_FRONTEND_SUBMAP_COLLECTION_VOXGRAPH_SUBMAP_COLLECTION_H_
#define VOXGRAPH_FRONTEND_SUBMAP_COLLECTION_VOXGRAPH_SUBMAP_COLLECTION_H_

#include <cblox/core/common.h>
#include <cblox/core/submap_collection.h>
#include <geometry_msgs/PoseStamped.h>
#include <minkindr_conversions/kindr_msg.h>
#include <ros/ros.h>
#include <voxblox/core/common.h>
#include <memory>
#include <utility>
#include <vector>
#include "voxgraph/common.h"
#include "voxgraph/frontend/submap_collection/submap_timeline.h"
#include "voxgraph/frontend/submap_collection/voxgraph_submap.h"

namespace voxgraph {
class VoxgraphSubmapCollection
    : public cblox::SubmapCollection<VoxgraphSubmap> {
 public:
  typedef std::shared_ptr<VoxgraphSubmapCollection> Ptr;
  typedef std::shared_ptr<const VoxgraphSubmapCollection> ConstPtr;
  typedef std::vector<geometry_msgs::PoseStamped> PoseStampedVector;

  explicit VoxgraphSubmapCollection(VoxgraphSubmap::Config submap_config,
                                    bool verbose = false)
      : SubmapCollection(submap_config),
        submap_creation_interval_(20),
        verbose_(verbose) {}

  void setSubmapCreationInterval(ros::Duration submap_creation_interval) {
    submap_creation_interval_ = std::move(submap_creation_interval);
  }

  bool shouldCreateNewSubmap(const ros::Time &current_time);

  // Overriden method that guarantees that the submap gets added to the timeline
  void createNewSubmap(const Transformation &T_world_robot,
                       const ros::Time &timestamp);
  // Delete the inherited methods to avoid accidental calls
  void createNewSubmap(const Transformation &T_M_S,
                       const SubmapID submap_id) = delete;
  SubmapID createNewSubmap(const Transformation &T_M_S) = delete;

  SubmapID getPreviousSubmapId() {
    return submap_timeline_.getPreviousSubmapId();
  }

  bool lookupActiveSubmapByTime(const ros::Time &timestamp,
                                SubmapID *submap_id) {
    return submap_timeline_.lookupActiveSubmapByTime(timestamp, submap_id);
  }

  PoseStampedVector getPoseHistory() const;

 private:
  bool verbose_;

  // Length of the time interval between the creation of subsequent submaps
  ros::Duration submap_creation_interval_;  // In seconds

  // Timeline to enable lookups of submaps by time
  SubmapTimeline submap_timeline_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_FRONTEND_SUBMAP_COLLECTION_VOXGRAPH_SUBMAP_COLLECTION_H_
