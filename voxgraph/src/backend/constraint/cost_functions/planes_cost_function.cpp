#include "voxgraph/backend/constraint/cost_functions/planes_cost_function.h"

#include <utility>

#include <kindr/minimal/quat-transformation.h>
#include <kindr/minimal/rotation-quaternion.h>

namespace voxgraph {

PlanesCostFunction::PlanesCostFunction(
    const Transformation& T_M_R_origin, const PlaneType::ConstPtr origin_plane,
    const Transformation& T_M_R_destination,
    const PlaneType::ConstPtr destination_plane,
    const Constraint::InformationMatrix& sqrt_information_matrix,
    const Config& config)
    : T_M_R_origin_init_(T_M_R_origin),
      T_M_R_destination_init_(T_M_R_destination),
      sqrt_information_matrix_(sqrt_information_matrix),
      origin_plane_(origin_plane),
      destination_plane_(destination_plane),
      config_(config) {
  CHECK(origin_plane);
  CHECK(destination_plane);
  Transformation T_M_P_origin_ = origin_plane->getPlaneTransformation();
  T_P_R_origin_ = T_M_P_origin_.inverse() * T_M_R_origin_init_;
  Transformation T_M_P_destination_ =
      destination_plane->getPlaneTransformation();
  T_P_R_destination_ = T_M_P_destination_.inverse() * T_M_R_destination_init_;
  // Set number of parameters: namely 2 poses, each having 4 params
  // (X,Y,Z,Yaw)
  // mutable_parameter_block_sizes()->clear();
  // mutable_parameter_block_sizes()->push_back(4);
  // mutable_parameter_block_sizes()->push_back(4);
  // set num residuals to 4
  // for n_diff_x,n_diff_y,n_diff_z,k_diff
  // set_num_residuals(4);
}

template <typename T>
Eigen::Matrix<T, 3, 3> rotationMatrixFromYaw(T yaw_radians) {
  const T cos_yaw = ceres::cos(yaw_radians);
  const T sin_yaw = ceres::sin(yaw_radians);
  const T zero = static_cast<T>(0);
  const T one = static_cast<T>(1);

  Eigen::Matrix<T, 3, 3> rotation_matrix;
  rotation_matrix << cos_yaw, -sin_yaw, zero, sin_yaw, cos_yaw, zero, zero,
      zero, one;
  return rotation_matrix;
}

bool PlanesCostFunction::Evaluate(double const* const* parameters,
                                  double* residuals, double** jacobians) const {
  typedef kindr::minimal::QuatTransformationTemplate<double> TransfT;
  const double* pose_A = parameters[0];
  const double* pose_B = parameters[1];
  const Eigen::Vector3d t_odom_A(pose_A[0], pose_A[1], pose_A[2]);
  const Eigen::Vector3d t_odom_B(pose_B[0], pose_B[1], pose_B[2]);
  const double yaw_odom_A(pose_A[3]);
  const double yaw_odom_B(pose_B[3]);
  CHECK_NEAR(yaw_odom_A, static_cast<double>(0.0), static_cast<double>(M_PI));
  CHECK_NEAR(yaw_odom_B, static_cast<double>(0.0), static_cast<double>(M_PI));

  Eigen::Map<Eigen::Matrix<double, 4, 1>> residuals_map(residuals);
  // const Eigen::Matrix<T, 1, 3> cc = (Eigen::Matrix<T, 1, 3>() << 0.0,
  // 0.0, 1.0); compute the transformations ^T_M_P for origin
  TransfT T_M_R_origin_init_casted = T_M_R_origin_init_.cast<double>();
  TransfT T_P_R_origin_casted = T_P_R_origin_.cast<double>();
  const Eigen::Matrix<double, 3, 3> RA = rotationMatrixFromYaw(yaw_odom_A);
  TransfT T_M_R_A_hat = TransfT(t_odom_A, TransfT::Rotation(RA));
  TransfT T_M_P_A_hat = T_M_R_A_hat * T_P_R_origin_casted.inverse();
  const TransfT::RotationMatrix& R_M_P_A_hat = T_M_P_A_hat.getRotationMatrix();
  // computa nA, kA
  Eigen::Matrix<double, 3, 1> nA = R_M_P_A_hat.col(2);
  double kA = nA.transpose() * T_M_P_A_hat.getPosition();
  // compute the transformation ^T_M_P for destination
  TransfT T_M_R_destination_init_casted =
      T_M_R_destination_init_.cast<double>();
  TransfT T_P_R_destination_casted = T_P_R_destination_.cast<double>();
  const Eigen::Matrix<double, 3, 3> RB = rotationMatrixFromYaw(yaw_odom_B);
  TransfT T_M_R_B_hat = TransfT(t_odom_B, TransfT::Rotation(RB));
  TransfT T_R_R_B_hat = T_M_R_destination_init_casted.inverse() * T_M_R_B_hat;
  TransfT T_M_P_B_hat = T_M_R_B_hat * T_P_R_destination_casted.inverse();
  const TransfT::RotationMatrix& R_M_P_B_hat = T_M_P_B_hat.getRotationMatrix();
  // compute nB, kB
  Eigen::Matrix<double, 3, 1> nB = R_M_P_B_hat.col(2);
  double kB = nB.transpose() * T_M_P_B_hat.getPosition();
  // compute normals residual (n1-n2)**2
  Eigen::Vector3d n_diff = nB - nA;
  residuals_map.template head<3>() = n_diff.cwiseProduct(n_diff);
  // compute offsets residual (k1-k2)**2
  double kdiff = kB - kA;
  residuals_map(3) = kdiff * kdiff;
  // Scale the residuals by the square root information matrix to account for
  // the measurement uncertainty
  residuals_map = sqrt_information_matrix_.cast<double>() * residuals_map;
  LOG(ERROR) << "residuals_map:\n" << residuals_map;
  // compute jacobians
  // Let's pray before we start doing this:
  // God, teach me and give me spiritual discernment. May your
  // Holy Spirit and your word direct me on the right path. Guard me
  // from false teachings and incorrect understanding of your
  // nature and your will.
  if (jacobians != nullptr) {
    Eigen::Vector3d cc;
    cc << 0.0, 0.0, 1.0;
    Eigen::Matrix3d Sz;
    Sz << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    // Eigen::Vector3d dp_dyaw = Eigen::Vector3d::Zero(3);
    // jacobians[0] for poseA
    // poseA has pA and yawA
    if (jacobians[0] != nullptr) {
      Eigen::Matrix3d dR_A_dyaw = Sz * RA;
      TransformationD constant_term = T_P_R_origin_casted.inverse();
      Eigen::Matrix4d dT_A_dpx = Eigen::Matrix4d::Zero();
      dT_A_dpx.block<1, 3>(0, 3) << 1.0, 0.0, 0.0;
      // LOG(ERROR) << "dT_A_dpx:\n" << dT_A_dpx;
      Eigen::Matrix4d dT_A_dpy = Eigen::Matrix4d::Zero();
      dT_A_dpy.block<1, 3>(0, 3) << 0.0, 1.0, 0.0;
      Eigen::Matrix4d dT_A_dpz = Eigen::Matrix4d::Zero();
      dT_A_dpz.block<1, 3>(0, 3) << 0.0, 0.0, 1.0;
      Eigen::Matrix4d dT_A_dyaw = Eigen::Matrix4d::Zero();
      dT_A_dyaw.block<3, 3>(0, 0) = dR_A_dyaw;
      Eigen::Matrix4d dT_M_P_hat_dpx =
          dT_A_dpx * constant_term.getTransformationMatrix();
      // LOG(ERROR) << "dT_M_P_hat_dpx:\n" << dT_M_P_hat_dpx;
      Eigen::Matrix4d dT_M_P_hat_dpy =
          dT_A_dpy * constant_term.getTransformationMatrix();
      Eigen::Matrix4d dT_M_P_hat_dpz =
          dT_A_dpz * constant_term.getTransformationMatrix();
      Eigen::Matrix4d dT_M_P_hat_dyaw =
          dT_A_dyaw * constant_term.getTransformationMatrix();
      Eigen::Vector3d dnA_dpx = dT_M_P_hat_dpx.block<3, 3>(0, 0) * cc;
      // LOG(ERROR) << "dnA_dpx:\n" << dnA_dpx;
      Eigen::Vector3d dnA_dpy = dT_M_P_hat_dpy.block<3, 3>(0, 0) * cc;
      Eigen::Vector3d dnA_dpz = dT_M_P_hat_dpz.block<3, 3>(0, 0) * cc;
      Eigen::Vector3d dnA_dyaw = dT_M_P_hat_dyaw.block<3, 3>(0, 0) * cc;
      double dkA_dpx = dT_M_P_hat_dpx.block<1, 3>(0, 3).dot(nA) +
                       T_M_P_A_hat.getPosition().dot(dnA_dpx);
      double dkA_dpy = dT_M_P_hat_dpy.block<1, 3>(0, 3).dot(nA) +
                       T_M_P_A_hat.getPosition().dot(dnA_dpy);
      double dkA_dpz = dT_M_P_hat_dpz.block<1, 3>(0, 3).dot(nA) +
                       T_M_P_A_hat.getPosition().dot(dnA_dpz);
      double dkA_dyaw = dT_M_P_hat_dyaw.block<1, 3>(0, 3).dot(nA) +
                        T_M_P_A_hat.getPosition().dot(dnA_dyaw);
      Eigen::Vector3d dn_diff_dpxA = -n_diff.cwiseProduct(dnA_dpx);
      // LOG(ERROR) << "dn_diff_dpxA:\n" << dn_diff_dpxA;
      Eigen::Vector3d dn_diff_dpyA = -n_diff.cwiseProduct(dnA_dpy);
      Eigen::Vector3d dn_diff_dpzA = -n_diff.cwiseProduct(dnA_dpz);
      Eigen::Vector3d dn_diff_dyawA = -n_diff.cwiseProduct(dnA_dyaw);
      double dk_diff_dpxA = -kdiff * dkA_dpx;
      double dk_diff_dpyA = -kdiff * dkA_dpy;
      double dk_diff_dpzA = -kdiff * dkA_dpz;
      double dk_diff_dyawA = -kdiff * dkA_dyaw;
      Eigen::Matrix4d jacobianMatA(jacobians[0]);
      // for n_diff 0
      jacobianMatA.block<3, 1>(0, 0) = dn_diff_dpxA;
      jacobianMatA.block<3, 1>(0, 1) = dn_diff_dpyA;
      jacobianMatA.block<3, 1>(0, 2) = dn_diff_dpzA;
      jacobianMatA.block<3, 1>(0, 3) = dn_diff_dyawA;
      // jacobians[0][0*4+0] = dn_diff_dpxA.x();
      // jacobians[0][0*4+1] = dn_diff_dpyA.x();
      // jacobians[0][0*4+2] = dn_diff_dpzA.x();
      // jacobians[0][0*4+3] = dn_diff_dyawA.x();
      // for n_diff 1
      // jacobians[0][1*4+0] = dn_diff_dpxA.y();
      // jacobians[0][1*4+1] = dn_diff_dpyA.y();
      // jacobians[0][1*4+2] = dn_diff_dpzA.y();
      // jacobians[0][1*4+3] = dn_diff_dyawA.y();
      // for n_diff 2
      // jacobians[0][2*4+0] = dn_diff_dpxA.z();
      // jacobians[0][2*4+1] = dn_diff_dpyA.z();
      // jacobians[0][2*4+2] = dn_diff_dpzA.z();
      // jacobians[0][2*4+3] = dn_diff_dyawA.z();
      // for k_diff
      jacobians[0][3 * 4 + 0] = dk_diff_dpxA;
      jacobians[0][3 * 4 + 1] = dk_diff_dpyA;
      jacobians[0][3 * 4 + 2] = dk_diff_dpzA;
      jacobians[0][3 * 4 + 3] = dk_diff_dyawA;
      LOG(ERROR) << "jacobians[0]:\n" << jacobianMatA;
    }
    // jacobians[1] for poseB
    // poseB has pB and yawB
    if (jacobians[1] != nullptr) {
      Eigen::Matrix3d dR_B_dyaw = Sz * RB;
      Eigen::Matrix4d constant_term =
          T_P_R_destination_casted.inverse().getTransformationMatrix();
      Eigen::Matrix4d dT_B_dpx = Eigen::Matrix4d::Zero();
      dT_B_dpx.block<1, 3>(0, 3) << 1.0, 0.0, 0.0;
      Eigen::Matrix4d dT_B_dpy = Eigen::Matrix4d::Zero();
      dT_B_dpy.block<1, 3>(0, 3) << 0.0, 1.0, 0.0;
      Eigen::Matrix4d dT_B_dpz = Eigen::Matrix4d::Zero();
      dT_B_dpz.block<1, 3>(0, 3) << 0.0, 0.0, 1.0;
      Eigen::Matrix4d dT_B_dyaw = Eigen::Matrix4d::Zero();
      dT_B_dyaw.block<3, 3>(0, 0) = dR_B_dyaw;
      Eigen::Matrix4d dT_M_P_hat_dpx = dT_B_dpx * constant_term;
      Eigen::Matrix4d dT_M_P_hat_dpy = dT_B_dpy * constant_term;
      Eigen::Matrix4d dT_M_P_hat_dpz = dT_B_dpz * constant_term;
      Eigen::Matrix4d dT_M_P_hat_dyaw = dT_B_dyaw * constant_term;
      Eigen::Vector3d dnB_dpx = dT_M_P_hat_dpx.block<3, 3>(0, 0) * cc;
      Eigen::Vector3d dnB_dpy = dT_M_P_hat_dpy.block<3, 3>(0, 0) * cc;
      Eigen::Vector3d dnB_dpz = dT_M_P_hat_dpz.block<3, 3>(0, 0) * cc;
      Eigen::Vector3d dnB_dyaw = dT_M_P_hat_dyaw.block<3, 3>(0, 0) * cc;
      double dkB_dpx = dT_M_P_hat_dpx.block<1, 3>(0, 3).dot(nB) +
                       T_M_P_B_hat.getPosition().dot(dnB_dpx);
      double dkB_dpy = dT_M_P_hat_dpy.block<1, 3>(0, 3).dot(nB) +
                       T_M_P_B_hat.getPosition().dot(dnB_dpy);
      double dkB_dpz = dT_M_P_hat_dpz.block<1, 3>(0, 3).dot(nB) +
                       T_M_P_B_hat.getPosition().dot(dnB_dpz);
      double dkB_dyaw = dT_M_P_hat_dyaw.block<1, 3>(0, 3).dot(nB) +
                        T_M_P_B_hat.getPosition().dot(dnB_dyaw);
      Eigen::Vector3d dn_diff_dpxB = n_diff.cwiseProduct(dnB_dpx);
      Eigen::Vector3d dn_diff_dpyB = n_diff.cwiseProduct(dnB_dpy);
      Eigen::Vector3d dn_diff_dpzB = n_diff.cwiseProduct(dnB_dpz);
      Eigen::Vector3d dn_diff_dyawB = n_diff.cwiseProduct(dnB_dyaw);
      double dk_diff_dpxB = kdiff * dkB_dpx;
      double dk_diff_dpyB = kdiff * dkB_dpy;
      double dk_diff_dpzB = kdiff * dkB_dpz;
      double dk_diff_dyawB = kdiff * dkB_dyaw;
      Eigen::Matrix4d jacobianMatB(jacobians[1]);
      jacobianMatB.block<3, 1>(0, 0) = dn_diff_dpxB;
      jacobianMatB.block<3, 1>(0, 1) = dn_diff_dpyB;
      jacobianMatB.block<3, 1>(0, 2) = dn_diff_dpzB;
      jacobianMatB.block<3, 1>(0, 3) = dn_diff_dyawB;
      // for n_diff 0
      // jacobians[1][0*4+0] = dn_diff_dpxB.x();
      // jacobians[1][0*4+1] = dn_diff_dpyB.x();
      // jacobians[1][0*4+2] = dn_diff_dpzB.x();
      // jacobians[1][0*4+3] = dn_diff_dyawB.x();
      // for n_diff 1
      // jacobians[1][1*4+0] = dn_diff_dpxB.y();
      // jacobians[1][1*4+1] = dn_diff_dpyB.y();
      // jacobians[1][1*4+2] = dn_diff_dpzB.y();
      // jacobians[1][1*4+3] = dn_diff_dyawB.y();
      // for n_diff 2
      // jacobians[1][2*4+0] = dn_diff_dpxB.z();
      // jacobians[1][2*4+1] = dn_diff_dpyB.z();
      // jacobians[1][2*4+2] = dn_diff_dpzB.z();
      // jacobians[1][2*4+3] = dn_diff_dyawB.z();
      // for k_diff
      jacobians[1][3 * 4 + 0] = dk_diff_dpxB;
      jacobians[1][3 * 4 + 1] = dk_diff_dpyB;
      jacobians[1][3 * 4 + 2] = dk_diff_dpzB;
      jacobians[1][3 * 4 + 3] = dk_diff_dyawB;
      LOG(ERROR) << "jacobians[1]:\n" << Eigen::Matrix4d(jacobians[0]);
    }
  }
  LOG(ERROR) << "PlanesCostFunction::Evaluate(...)\n";

  return true;
}
/*
template <typename T>
bool PlanesCostFunction::operator()(const T* const pose_A,
                                    const T* const pose_B, T* residuals) const {
  // const Eigen::Matrix<T, 4, 1> t_odom_A(pose_A[0], pose_A[1], pose_A[2],
  //                                       static_cast<T>(1.0));
  // const Eigen::Matrix<T, 4, 1> t_odom_B(pose_B[0], pose_B[1], pose_B[2],
  //                                       static_cast<T>(1.0));
  // const T yaw_odom_A(pose_A[3]);
  // const T yaw_odom_B(pose_B[3]);
  // CHECK_NEAR(yaw_odom_A, static_cast<T>(0.0), static_cast<T>(M_PI));
  // CHECK_NEAR(yaw_odom_B, static_cast<T>(0.0), static_cast<T>(M_PI));
  // for A
  LOG(ERROR) << "COMPUTED TILL A";
  // Eigen::Matrix<T, 3, 3> RA = rotationMatrixFromYaw(yaw_odom_A);
  // Eigen::Matrix<T, 4, 4> T_M_R_A_hat = Eigen::Matrix<T, 4, 4>::Zero();
  // T_M_R_A_hat.block(0, 0, 3, 3) = RA;
  // T_M_R_A_hat.block(0, 3, 3, 1) = t_odom_A;
  // T_M_R_A_hat(3, 3) = static_cast<T>(1.0);
  // Eigen::Matrix<T, 4, 4> T_M_P_A_hat =
  //     T_M_R_A_hat * T_P_R_origin_.inverse().getTransformationMatrix().cast<T>();
  Eigen::Matrix<T, 3, 1> nA = Eigen::Matrix<T, 3, 1>::Zero();//std::move(T_M_P_A_hat.block(0, 2, 3, 1));
  LOG(ERROR) << "COMPUTED TILL B";
  // T kA = nA.transpose() * (T_M_P_A_hat.block(0,3,3,1));
  // T kA = nA.data()[0] * T_M_P_A_hat.data()[3 * 4 + 0] +
  //        nA.data()[1] * T_M_P_A_hat.data()[3 * 4 + 1] +
  //        nA.data()[2] * T_M_P_A_hat.data()[3 * 4 + 2];
  T kA = static_cast<T> (0);
  // for B
  // Eigen::Matrix<T, 3, 3> RB = rotationMatrixFromYaw(yaw_odom_B);
  // Eigen::Matrix<T, 4, 4> T_M_R_B_hat = Eigen::Matrix<T, 4, 4>::Zero();
  // T_M_R_B_hat.block(0, 0, 3, 3) = RB;
  // T_M_R_B_hat.block(0, 3, 3, 1) = t_odom_B;
  // T_M_R_B_hat(3, 3) = static_cast<T>(1.0);
  // Eigen::Matrix<T, 4, 4> T_M_P_B_hat =
  //     T_M_R_B_hat *
  //     T_P_R_destination_.inverse().getTransformationMatrix().cast<T>();
  Eigen::Matrix<T, 3, 1> nB = Eigen::Matrix<T, 3, 1>::Zero();//std::move(T_M_P_B_hat.block(0, 2, 3, 1));
  // T kB = nB.data()[0] * T_M_P_B_hat.data()[3 * 4 + 0] +
  //        nB.data()[1] * T_M_P_B_hat.data()[3 * 4 + 1] +
  //        nB.data()[2] * T_M_P_B_hat.data()[3 * 4 + 2];
  T kB = static_cast<T>(0);
  // Eigen::Map<Eigen::Matrix<T, 2, 1>> residuals_map(residuals);
  LOG(ERROR) << "COMPUTED TILL C";
  // residuals[0] = (nA - nB).transpose().dot(nA - nB);
  // residuals[1] = (kA - kB) * (kA - kB);
  LOG(ERROR) << "COMPUTED TILL D";
}
*/

ceres::CostFunction* PlanesCostFunction::Create(
    const Transformation& T_M_R_origin, const PlaneType::ConstPtr origin_plane,
    const Transformation& T_M_R_destination,
    const PlaneType::ConstPtr destination_plane,
    const Constraint::InformationMatrix& sqrt_information_matrix,
    const PlanesCostFunction::Config& planes_cost_config) {
  return (new ceres::AutoDiffCostFunction<PlanesCostFunction, 2, 4, 4>(
      new PlanesCostFunction(T_M_R_origin, origin_plane, T_M_R_destination,
                             destination_plane, sqrt_information_matrix,
                             planes_cost_config)));
}

}  // namespace voxgraph