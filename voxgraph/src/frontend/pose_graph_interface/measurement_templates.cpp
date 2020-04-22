#include "voxgraph/frontend/pose_graph_interface/measurement_templates.h"

#include <string>

namespace voxgraph {
MeasurementTemplates::MeasurementTemplates(bool verbose)
    : odometry(odometry_),
      loop_closure(loop_closure_),
      gps(gps_),
      height(height_),
      registration(registration_),
      verbose_(verbose) {
  // Initialize the default odometry measurement config
  odometry_.information_matrix.setZero();

  // Initialize the default loop closure config
  loop_closure_.information_matrix.setZero();

  // Initialize the default GPS measurement config
  gps_.information_matrix.setZero();
  gps_.reference_frame_id = NodeTemplates::kGpsFrame;
  // NOTE: We allow semi definite information matrices s.t. an infinite
  //       covariance on Yaw (e.g. a zero in the information matrix) can be used
  gps_.allow_semi_definite_information_matrix = true;

  // Initialize the default height measurement config
  height_.information_matrix.setZero();
  height_.reference_frame_id = NodeTemplates::kMissionFrame;
  height_.allow_semi_definite_information_matrix = true;

  // Initialize the default registration config
  registration_.information_matrix.setZero();
}

void MeasurementTemplates::setFromRosParams(
    const ros::NodeHandle& node_handle) {
  if (node_handle.hasParam("odometry")) {
    // Set the information matrix
    setInformationMatrixFromRosParams(
        ros::NodeHandle(node_handle, "odometry/information_matrix"),
        &odometry_.information_matrix);
    ROS_INFO_STREAM_COND(verbose_, "Setting odometry information matrix to:\n"
                                       << odometry.information_matrix);
  }

  if (node_handle.hasParam("loop_closure")) {
    // Set the information matrix
    setInformationMatrixFromRosParams(
        ros::NodeHandle(node_handle, "loop_closure/information_matrix"),
        &loop_closure_.information_matrix);
    ROS_INFO_STREAM_COND(verbose_,
                         "Setting loop closure information matrix "
                         "to:\n"
                             << loop_closure.information_matrix);
  }

  if (node_handle.hasParam("gps")) {
    // Set the information matrix
    setInformationMatrixFromRosParams(
        ros::NodeHandle(node_handle, "gps/information_matrix"),
        &gps_.information_matrix);
    ROS_INFO_STREAM_COND(verbose_,
                         "Setting gps measurement information "
                         "matrix to:\n"
                             << gps.information_matrix);
  }

  if (node_handle.hasParam("height")) {
    // Set the information matrix
    height_.information_matrix.setZero();
    // NOTE: We model height constraints as absolute pose constraints with
    //       infinite covariance on X, Y, Yaw. The information matrix therefore
    //       consists entirely of zeros except for the Z (height) entry.
    node_handle.param("height/information_zz", height_.information_matrix(2, 2),
                      0.0);
    ROS_INFO_STREAM_COND(verbose_,
                         "Setting height measurement information matrix to:\n"
                             << height.information_matrix);
  }

  if (node_handle.hasParam("submap_registration")) {
    // Set the sampling ratio
    node_handle.param("submap_registration/sampling_ratio",
                      registration_.registration.sampling_ratio,
                      registration_.registration.sampling_ratio);
    ROS_INFO_STREAM_COND(verbose_,
                         "Setting submap registration sampling ratio "
                         "to: "
                             << registration.registration.sampling_ratio);

    // Set the registration method (from string)
    {
      std::string registration_method_str;
      node_handle.param<std::string>("submap_registration/registration_method",
                                     registration_method_str,
                                     "implicit_to_implicit");
      if (registration_method_str == "implicit_to_implicit") {
        registration_.registration.registration_point_type =
            VoxgraphSubmap::RegistrationPointType::kVoxels;
      } else if (registration_method_str == "explicit_to_implicit") {
        registration_.registration.registration_point_type =
            VoxgraphSubmap::RegistrationPointType::kIsosurfacePoints;
      } else {
        ROS_WARN_STREAM(
            "Param \"submap_registration/registration_method\" must be "
            "\"implicit_to_implicit\" (default) or \"explicit_to_implicit\", "
            "but received \""
            << registration_method_str
            << "\". "
               "Will use default instead.");
      }
      ROS_INFO_STREAM_COND(verbose_, "Setting submap registration method to: "
                                         << registration_method_str);
    }

    // Set the information matrix
    setInformationMatrixFromRosParams(
        ros::NodeHandle(node_handle, "submap_registration/information_matrix"),
        &registration_.information_matrix);
    ROS_INFO_STREAM_COND(verbose_,
                         "Setting submap registration information matrix to:\n"
                             << registration.information_matrix);
  }
}

void MeasurementTemplates::setInformationMatrixFromRosParams(
    const ros::NodeHandle& node_handle,
    Constraint::InformationMatrix* information_matrix) {
  CHECK_NOTNULL(information_matrix);
  Constraint::InformationMatrix& information_matrix_ref = *information_matrix;

  // Set the upper triangular part of the information matrix from ROS params
  node_handle.param("x_x", information_matrix_ref(0, 0), 0.0);
  node_handle.param("x_y", information_matrix_ref(0, 1), 0.0);
  node_handle.param("x_z", information_matrix_ref(0, 2), 0.0);
  node_handle.param("x_yaw", information_matrix_ref(0, 3), 0.0);

  node_handle.param("y_y", information_matrix_ref(1, 1), 0.0);
  node_handle.param("y_z", information_matrix_ref(1, 2), 0.0);
  node_handle.param("y_yaw", information_matrix_ref(1, 3), 0.0);

  node_handle.param("z_z", information_matrix_ref(2, 2), 0.0);
  node_handle.param("z_yaw", information_matrix_ref(2, 3), 0.0);

  node_handle.param("yaw_yaw", information_matrix_ref(3, 3), 0.0);

  // Copy the upper to the lower triangular part, to get a symmetric info matrix
  information_matrix_ref =
      information_matrix_ref.selfadjointView<Eigen::Upper>();
}
}  // namespace voxgraph
