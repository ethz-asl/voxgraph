#include "voxgraph/frontend/submap_collection/submap_timeline.h"

namespace voxgraph {
void voxgraph::SubmapTimeline::addNextSubmap(
    const ros::Time& submap_end_timestamp, const cblox::SubmapID& submap_id) {
  submap_timeline_.emplace_hint(submap_timeline_.end(), submap_end_timestamp,
                                submap_id);
}

bool SubmapTimeline::lookupActiveSubmapByTime(const ros::Time& timestamp,
                                              cblox::SubmapID* submap_id) {
  CHECK_NOTNULL(submap_id);

  // Get an iterator to the end of the time interval in which timestamp falls
  auto iterator = submap_timeline_.upper_bound(timestamp);

  // Ensure that the timestamp does not fall after the last submap
  if (iterator == submap_timeline_.end()) {
    return false;
  }

  // The interval's active submap id is stored at its end point
  *submap_id = iterator->second;
  return true;
}

cblox::SubmapID SubmapTimeline::getPreviousSubmapId() const {
  CHECK_GE(submap_timeline_.size(), 2);
  return (++submap_timeline_.crbegin())->second;
}

cblox::SubmapID SubmapTimeline::getFirstSubmapId() const {
  CHECK(!submap_timeline_.empty());
  return submap_timeline_.cbegin()->second;
}

cblox::SubmapID SubmapTimeline::getLastSubmapId() const {
  CHECK(!submap_timeline_.empty());
  return submap_timeline_.crbegin()->second;
}
}  // namespace voxgraph
