#ifndef VOXGRAPH_TOOLS_IO_H_
#define VOXGRAPH_TOOLS_IO_H_

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <rosbag/bag.h>

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
  return true;  // Zero error checking here!
}

template <typename T>
bool saveVectorToFile(const std::string& filepath, const std::vector<T> vec) {
  std::ofstream os;
  os.open(filepath, std::ios::out | std::ios::trunc);
  for (const auto& num : vec) {
    os << num << std::endl;
  }
  os.close();
  return true;
}
}  // namespace io
}  // namespace voxgraph

#endif  // VOXGRAPH_TOOLS_IO_H_
