#include "voxgraph/frontend/submap_collection/submap_timeline.h"

namespace voxgraph {
void voxgraph::SubmapTimeline::addNextSubmap(
    const ros::Time& submap_end_timestamp, const cblox::SubmapID& submap_id) {
  submap_timeline_.emplace_hint(submap_timeline_.end(), submap_end_timestamp,
                                submap_id);
}

bool SubmapTimeline::lookupActiveSubmapByTime(
    const ros::Time& timestamp, cblox::SubmapID* submap_id) const {
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

bool SubmapTimeline::getPreviousSubmapId(SubmapID* submap_id) const {
  CHECK_NOTNULL(submap_id);
  if (1 < submap_timeline_.size()) {
    *submap_id = (++submap_timeline_.crbegin())->second;
    return true;
  }
  return false;
}

bool SubmapTimeline::getFirstSubmapId(SubmapID* submap_id) const {
  CHECK_NOTNULL(submap_id);
  if (!submap_timeline_.empty()) {
    *submap_id = submap_timeline_.cbegin()->second;
    return true;
  }
  return false;
}

bool SubmapTimeline::getLastSubmapId(SubmapID* submap_id) const {
  CHECK_NOTNULL(submap_id);
  if (!submap_timeline_.empty()) {
    *submap_id = submap_timeline_.crbegin()->second;
    return true;
  }
  return false;
}
}  // namespace voxgraph
