#ifndef VOXGRAPH_FRONTEND_POSE_GRAPH_INTERFACE_MEASUREMENT_TEMPLATES_H_
#define VOXGRAPH_FRONTEND_POSE_GRAPH_INTERFACE_MEASUREMENT_TEMPLATES_H_

#include "voxgraph/backend/constraint/absolute_pose_constraint.h"
#include "voxgraph/backend/constraint/registration_constraint.h"
#include "voxgraph/backend/constraint/relative_pose_constraint.h"
#include "voxgraph/frontend/pose_graph_interface/node_templates.h"

namespace voxgraph {
class MeasurementTemplates {
 public:
  explicit MeasurementTemplates(bool verbose = false);

  void setFromRosParams(const ros::NodeHandle& node_handle);

  const RelativePoseConstraint::Config& odometry;
  const RelativePoseConstraint::Config& loop_closure;
  const AbsolutePoseConstraint::Config& gps;
  const AbsolutePoseConstraint::Config& height;
  const RegistrationConstraint::Config& registration;

 private:
  bool verbose_;

  void setInformationMatrixFromRosParams(
      const ros::NodeHandle& node_handle,
      Constraint::InformationMatrix* information_matrix);

  RelativePoseConstraint::Config odometry_;
  RelativePoseConstraint::Config loop_closure_;
  AbsolutePoseConstraint::Config gps_;
  AbsolutePoseConstraint::Config height_;
  RegistrationConstraint::Config registration_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_FRONTEND_POSE_GRAPH_INTERFACE_MEASUREMENT_TEMPLATES_H_
