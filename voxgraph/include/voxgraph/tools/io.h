#ifndef VOXGRAPH_TOOLS_IO_H_
#define VOXGRAPH_TOOLS_IO_H_

#include <rosbag/bag.h>
#include <string>
#include "voxgraph/frontend/submap_collection/voxgraph_submap_collection.h"

namespace voxgraph {
namespace io {
bool savePoseHistoryToFile(
    const std::string& filepath,
    const VoxgraphSubmapCollection::PoseStampedVector& pose_history) {
  // Write the pose history to a rosbag
  rosbag::Bag bag;
  bag.open(filepath, rosbag::bagmode::Write);
  for (const geometry_msgs::PoseStamped& pose_stamped : pose_history) {
    bag.write("pose_history", pose_stamped.header.stamp, pose_stamped);
  }
  bag.close();
}
}  // namespace io
}  // namespace voxgraph

#endif  // VOXGRAPH_TOOLS_IO_H_
