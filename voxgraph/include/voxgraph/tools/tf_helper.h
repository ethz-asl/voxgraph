#ifndef VOXGRAPH_TOOLS_TF_HELPER_H_
#define VOXGRAPH_TOOLS_TF_HELPER_H_

#include <string>

#include <ros/ros.h>
#include <voxblox/core/common.h>

namespace voxgraph {
class TfHelper {
 public:
  static void publishTransform(const voxblox::Transformation& transform,
                               const std::string& base_frame,
                               const std::string& target_frame,
                               bool tf_is_static = false,
                               const ros::Time& timestamp = ros::Time::now());
};
}  // namespace voxgraph

#endif  // VOXGRAPH_TOOLS_TF_HELPER_H_
