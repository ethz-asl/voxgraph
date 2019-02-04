//
// Created by victor on 05.01.19.
//

#ifndef VOXGRAPH_UTILS_TF_HELPER_H_
#define VOXGRAPH_UTILS_TF_HELPER_H_

#include <ros/ros.h>
#include <voxblox/core/common.h>
#include <string>

namespace voxgraph {
class TfHelper {
 public:
  static void publishTransform(const voxblox::Transformation &transform,
                               const std::string &base_frame,
                               const std::string &target_frame,
                               bool tf_is_static = false,
                               const ros::Time &timestamp = ros::Time::now());
};
}  // namespace voxgraph

#endif  // VOXGRAPH_UTILS_TF_HELPER_H_
