#include "voxgraph/frontend/map_tracker/transformers/tf_transformer.h"

#include <string>

namespace voxgraph {
bool TfTransformer::waitForTransform(const std::string& to_frame_id,
                                     const std::string& from_frame_id,
                                     const ros::Time& frame_timestamp) {
  // Total time spent waiting for the updated pose
  ros::WallDuration t_waited(0.0);
  while (t_waited < transform_lookup_max_time_) {
    if (tf_buffer_.canTransform(to_frame_id, from_frame_id, frame_timestamp)) {
      return true;
    }
    transform_lookup_retry_period_.sleep();
    t_waited += transform_lookup_retry_period_;
  }
  ROS_WARN("Waited %.3fs, but still could not get the TF from %s to %s",
           t_waited.toSec(), from_frame_id.c_str(), to_frame_id.c_str());
  return false;
}

bool TfTransformer::lookupTransform(const std::string& to_frame_id,
                                    const std::string& from_frame_id,
                                    const ros::Time& frame_timestamp,
                                    Transformation* transform) {
  CHECK_NOTNULL(transform);
  if (!waitForTransform(to_frame_id, from_frame_id, frame_timestamp)) {
    return false;
  }
  geometry_msgs::TransformStamped transform_msg =
      tf_buffer_.lookupTransform(to_frame_id, from_frame_id, frame_timestamp);
  tf::transformMsgToKindr(transform_msg.transform, transform);
  return true;
}
}  // namespace voxgraph
