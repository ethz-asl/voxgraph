#ifndef VOXGRAPH_FRONTEND_SUBMAP_COLLECTION_SUBMAP_TIMELINE_H_
#define VOXGRAPH_FRONTEND_SUBMAP_COLLECTION_SUBMAP_TIMELINE_H_

#include <algorithm>
#include <map>
#include <vector>

#include <cblox/core/common.h>
#include <ros/time.h>

namespace voxgraph {
class SubmapTimeline {
 public:
  SubmapTimeline() = default;

  void addNextSubmap(const ros::Time& submap_creation_timestamp,
                     const cblox::SubmapID& submap_id);

  bool lookupActiveSubmapByTime(const ros::Time& timestamp,
                                cblox::SubmapID* submap_id);

  cblox::SubmapID getPreviousSubmapId() const;
  cblox::SubmapID getFirstSubmapId() const;
  cblox::SubmapID getLastSubmapId() const;

 private:
  // Map from each time interval's start time to the corresponding active submap
  std::map<const ros::Time, cblox::SubmapID> submap_timeline_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_FRONTEND_SUBMAP_COLLECTION_SUBMAP_TIMELINE_H_
