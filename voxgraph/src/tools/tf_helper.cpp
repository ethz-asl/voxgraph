#include "voxgraph/tools/tf_helper.h"

#include <string>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

namespace voxgraph {
void TfHelper::publishTransform(const voxblox::Transformation& transform,
                                const std::string& base_frame,
                                const std::string& target_frame,
                                bool tf_is_static, const ros::Time& timestamp) {
  static tf2_ros::TransformBroadcaster transform_broadcaster;
  static tf2_ros::StaticTransformBroadcaster static_transform_broadcaster;

  geometry_msgs::TransformStamped tf_stamped;
  tf_stamped.header.stamp = timestamp;
  tf_stamped.header.frame_id = base_frame;
  tf_stamped.child_frame_id = target_frame;
  tf_stamped.transform.translation.x = transform.getPosition().x();
  tf_stamped.transform.translation.y = transform.getPosition().y();
  tf_stamped.transform.translation.z = transform.getPosition().z();
  tf_stamped.transform.rotation.x = transform.getRotation().x();
  tf_stamped.transform.rotation.y = transform.getRotation().y();
  tf_stamped.transform.rotation.z = transform.getRotation().z();
  tf_stamped.transform.rotation.w = transform.getRotation().w();

  if (tf_is_static) {
    static_transform_broadcaster.sendTransform(tf_stamped);
  } else {
    transform_broadcaster.sendTransform(tf_stamped);
  }
}
}  // namespace voxgraph
