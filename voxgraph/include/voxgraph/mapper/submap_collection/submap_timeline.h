//
// Created by victor on 06.03.19.
//

#ifndef VOXGRAPH_MAPPER_SUBMAP_COLLECTION_SUBMAP_TIMELINE_H_
#define VOXGRAPH_MAPPER_SUBMAP_COLLECTION_SUBMAP_TIMELINE_H_

#include <cblox/core/common.h>
#include <ros/time.h>
#include <algorithm>
#include <map>
#include <vector>

namespace voxgraph {
class SubmapTimeline {
 public:
  SubmapTimeline() = default;

  void addNextSubmap(const ros::Time &submap_creation_timestamp,
                     const cblox::SubmapID &submap_id);

  bool lookupActiveSubmapByTime(const ros::Time &timestamp,
                                cblox::SubmapID *submap_id);

 private:
  // Map from each time interval's start time to the corresponding active submap
  std::map<const ros::Time, cblox::SubmapID> submap_timeline_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_MAPPER_SUBMAP_COLLECTION_SUBMAP_TIMELINE_H_
