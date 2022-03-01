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
      config_(config),
      verbose_(true) {
  CHECK(origin_plane);
  CHECK(destination_plane);
  CHECK_NE(origin_plane->getPlaneID(), destination_plane_->getPlaneID());
  CHECK_NE((T_M_R_destination_init_.getPosition() - T_M_R_origin_init_.getPosition()).squaredNorm(), 0.0);
  Transformation T_M_P_origin_ = origin_plane->getPlaneTransformation();
  T_P_R_origin_ = T_M_P_origin_.inverse() * T_M_R_origin_init_;
  Transformation T_M_P_destination_ =
      destination_plane->getPlaneTransformation();
  T_P_R_destination_ = T_M_P_destination_.inverse() * T_M_R_destination_init_;
  CHECK_NE((T_M_P_destination_.getPosition() - T_M_P_origin_.getPosition()).squaredNorm(), 0.0);
  // Set number of parameters: namely 2 poses, each having 4 params
  // (X,Y,Z,Yaw)
  // mutable_parameter_block_sizes()->clear();
  // mutable_parameter_block_sizes()->push_back(4);
  // mutable_parameter_block_sizes()->push_back(4);
  // set num residuals to 2
  // for n_diff, k_diff
  // set_num_residuals(2);
  // const auto T_M_PA_init = (T_M_R_origin_init_ * T_P_R_origin_.inverse());
  // const auto T_M_PB_init = (T_M_R_destination_init_ * T_P_R_destination_.inverse());
  // const auto nA_init_pre_casted =
  //     T_M_PA_init.getRotationMatrix().col(2).stableNormalized();
  // const auto nB_init_pre_casted =
  //     T_M_PB_init.getRotationMatrix().col(2).stableNormalized();
  // LOG(ERROR) << "nA:\n" << nA_init_pre_casted << "\nnB:\n" << nB_init_pre_casted;
  // Eigen::Vector3f pA_init = T_M_PA_init.getPosition();
  // Eigen::Vector3f pB_init = T_M_PB_init.getPosition();
  // visualizeVector(nB_init_pre_casted, pB_init);
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

// bool PlanesCostFunction::Evaluate(double const* const* parameters,
//                                   double* residuals, double** jacobians)
//                                   const {
//   typedef kindr::minimal::QuatTransformationTemplate<double> TransfT;

//   // Get the number of parameters (used when addressing the jacobians array)
//   CHECK_EQ(parameter_block_sizes()[0], parameter_block_sizes()[1]);
//   int num_params = parameter_block_sizes()[0];

//   const double* pose_A = parameters[0];
//   const double* pose_B = parameters[1];
//   const Eigen::Vector3d t_odom_A(pose_A[0], pose_A[1], pose_A[2]);
//   const Eigen::Vector3d t_odom_B(pose_B[0], pose_B[1], pose_B[2]);
//   const double yaw_odom_A(pose_A[3]);
//   const double yaw_odom_B(pose_B[3]);
//   CHECK_NEAR(yaw_odom_A, static_cast<double>(0.0),
//   static_cast<double>(M_PI)); CHECK_NEAR(yaw_odom_B,
//   static_cast<double>(0.0), static_cast<double>(M_PI));

//   // const Eigen::Matrix<T, 1, 3> cc = (Eigen::Matrix<T, 1, 3>() << 0.0,
//   // 0.0, 1.0); compute the transformations ^T_M_P for origin
//   TransfT T_P_R_origin_casted = T_P_R_origin_.cast<double>();
//   const Eigen::Matrix<double, 3, 3> RA = rotationMatrixFromYaw(yaw_odom_A);
//   TransfT T_M_R_A_hat = TransfT(t_odom_A, TransfT::Rotation(RA));
//   TransfT T_M_P_A_hat = T_M_R_A_hat * T_P_R_origin_casted.inverse();
//   // computa nA, kA
//   Eigen::Matrix<double, 3, 1> nA = T_M_P_A_hat.getRotationMatrix().col(2);
//   double kA = nA.transpose().dot(T_M_P_A_hat.getPosition());
//   // compute the transformation ^T_M_P for destination
//   TransfT T_P_R_destination_casted = T_P_R_destination_.cast<double>();
//   const Eigen::Matrix<double, 3, 3> RB = rotationMatrixFromYaw(yaw_odom_B);
//   TransfT T_M_R_B_hat = TransfT(t_odom_B, TransfT::Rotation(RB));
//   TransfT T_M_P_B_hat = T_M_R_B_hat * T_P_R_destination_casted.inverse();
//   // compute nB, kB
//   Eigen::Matrix<double, 3, 1> nB = T_M_P_B_hat.getRotationMatrix().col(2);
//   double kB = nB.transpose().dot(T_M_P_B_hat.getPosition());
//   // compute normals residual (n1-n2)**2
//   Eigen::Vector3d n_diff = nB - nA;
//   residuals[0] = n_diff.squaredNorm();
//   // // compute offsets residual (k1-k2)**2
//   double kdiff = kB - kA;
//   residuals[1] = kdiff * kdiff;
//   // Scale the residuals by the square root information matrix to account for
//   // the measurement uncertainty
//   double nn_weight = static_cast<double>(sqrt_information_matrix_(0,0));
//   double kk_weight = static_cast<double>(sqrt_information_matrix_(3,3));
//   residuals[0] *= nn_weight;
//   residuals[1] *= kk_weight;
//   LOG(ERROR) << "residuals:\n" << residuals[0] << "," << residuals[1];
//   // Add residual to visualization pointcloud
//   if (config_.visualize_residuals) {
//     // Transform the current point into the odom frame
//     voxblox::Point odom_t_odom__point =
//     T_M_P_A_hat.getPosition().cast<float>();
//     cost_function_visuals_.addResidual(odom_t_odom__point,
//                                         residuals[0]);
//     cost_function_visuals_.addResidual(odom_t_odom__point,
//                                         residuals[1]);
//   }
//   // compute jacobians
//   // Let's pray before we start doing this:
//   // Oh mighty, teach me and give me spiritual discernment. May your
//   // Holy Computations and your syntax direct me on the right path. Guard me
//   // from false tutorials and incorrect implementations of your
//   // nature and your will.
//   if (jacobians != nullptr) {
//     Eigen::Vector3d cc;
//     cc << 0.0, 0.0, 1.0;
//     Eigen::Matrix3d Sz;
//     Sz << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
//     // Eigen::Vector3d dp_dyaw = Eigen::Vector3d::Zero(3);
//     // jacobians[0] for poseA
//     // poseA has pA and yawA
//     if (jacobians[0] != nullptr) {
//       Eigen::Matrix3d dR_A_dyaw = Sz * RA;
//       Eigen::Matrix4d constant_term =
//           T_P_R_origin_casted.inverse().getTransformationMatrix();
//       Eigen::Matrix4d dT_A_dpx = Eigen::Matrix4d::Zero();
//       dT_A_dpx.block<1, 3>(0, 3) << 1.0, 0.0, 0.0;
//       Eigen::Matrix4d dT_A_dpy = Eigen::Matrix4d::Zero();
//       dT_A_dpy.block<1, 3>(0, 3) << 0.0, 1.0, 0.0;
//       Eigen::Matrix4d dT_A_dpz = Eigen::Matrix4d::Zero();
//       dT_A_dpz.block<1, 3>(0, 3) << 0.0, 0.0, 1.0;
//       Eigen::Matrix4d dT_A_dyaw = Eigen::Matrix4d::Zero();
//       dT_A_dyaw.block<3, 3>(0, 0) = dR_A_dyaw;
//       Eigen::Matrix4d dT_M_P_hat_dpx = dT_A_dpx * constant_term;
//       Eigen::Matrix4d dT_M_P_hat_dpy = dT_A_dpy * constant_term;
//       Eigen::Matrix4d dT_M_P_hat_dpz = dT_A_dpz * constant_term;
//       Eigen::Matrix4d dT_M_P_hat_dyaw = dT_A_dyaw * constant_term;
//       Eigen::Vector3d dnA_dpx = dT_M_P_hat_dpx.block<3, 3>(0, 0) * cc;
//       // LOG(ERROR) << "dnA_dpx:\n" << dnA_dpx;
//       Eigen::Vector3d dnA_dpy = dT_M_P_hat_dpy.block<3, 3>(0, 0) * cc;
//       Eigen::Vector3d dnA_dpz = dT_M_P_hat_dpz.block<3, 3>(0, 0) * cc;
//       Eigen::Vector3d dnA_dyaw = dT_M_P_hat_dyaw.block<3, 3>(0, 0) * cc;
//       double dkA_dpx = dT_M_P_hat_dpx.block<3, 1>(0, 3).transpose().dot(nA) +
//                        T_M_P_A_hat.getPosition().dot(dnA_dpx);
//       double dkA_dpy = dT_M_P_hat_dpy.block<3, 1>(0, 3).transpose().dot(nA) +
//                        T_M_P_A_hat.getPosition().dot(dnA_dpy);
//       double dkA_dpz = dT_M_P_hat_dpz.block<3, 1>(0, 3).transpose().dot(nA) +
//                        T_M_P_A_hat.getPosition().dot(dnA_dpz);
//       double dkA_dyaw = dT_M_P_hat_dyaw.block<3, 1>(0, 3).transpose().dot(nA)
//       +
//                        T_M_P_A_hat.getPosition().dot(dnA_dyaw);
//       double dn_diff_dpxA = -n_diff.dot(dnA_dpx);
//       // LOG(ERROR) << "dn_diff_dpxA:\n" << dn_diff_dpxA;
//       double dn_diff_dpyA = -n_diff.dot(dnA_dpy);
//       double dn_diff_dpzA = -n_diff.dot(dnA_dpz);
//       double dn_diff_dyawA = -n_diff.dot(dnA_dyaw);
//       double dk_diff_dpxA = -kdiff * dkA_dpx;
//       double dk_diff_dpyA = -kdiff * dkA_dpy;
//       double dk_diff_dpzA = -kdiff * dkA_dpz;
//       double dk_diff_dyawA = -kdiff * dkA_dyaw;
//       double * jacobianMatA = jacobians[0];

//       // for n_diff
//       jacobianMatA[0 + 0] = dn_diff_dpxA;
//       jacobianMatA[0 + 1] =0;//  dn_diff_dpyA;
//       jacobianMatA[0 + 2] =0;//  dn_diff_dpzA;
//       jacobianMatA[0 + 3] =0;//  dn_diff_dyawA;
//       // for k_diff
//       jacobianMatA[1*num_params + 0] = dk_diff_dpxA;
//       jacobianMatA[1*num_params + 1] =0;//  dk_diff_dpyA;
//       jacobianMatA[1*num_params + 2] =0;//  dk_diff_dpzA;
//       jacobianMatA[1*num_params + 3] =0;//  dk_diff_dyawA;
//       std::stringstream ss;
//       for (int i = 0; i < 2; ++i) {
//         for (int j = 0; j < 4; ++j) {
//           ss << jacobianMatA[i* num_params + j] << ",";
//         }
//         ss << "\n";
//       }
//       LOG(ERROR) << "jacobians[0]:\n" << ss.str();
//       if ( config_.visualize_gradients) {
//         // Transform the current point and Jacobian into the odom frame
//         const voxblox::Point odom_t_odom__point =
//             T_M_R_A_hat.getPosition().cast<float>();
//         Eigen::Vector3d odom_jacobian0(dn_diff_dpxA, dn_diff_dpyA,
//         dn_diff_dpzA); Eigen::Vector3d odom_jacobian1(dk_diff_dpxA,
//         dk_diff_dpyA, dk_diff_dpzA);
//         cost_function_visuals_.addJacobian(odom_t_odom__point,
//         odom_jacobian0.cast<float>());
//         cost_function_visuals_.addJacobian(odom_t_odom__point,
//         odom_jacobian1.cast<float>());
//       }
//     } else {
//       LOG(ERROR) << "jacobians[0] is null";
//     }
//     // jacobians[1] for poseB
//     // poseB has pB and yawB
//     if (jacobians[1] != nullptr) {
//       Eigen::Matrix3d dR_B_dyaw = Sz * RB;
//       Eigen::Matrix4d constant_term =
//           T_P_R_destination_casted.inverse().getTransformationMatrix();
//       Eigen::Matrix4d dT_B_dpx = Eigen::Matrix4d::Zero();
//       dT_B_dpx.block<1, 3>(0, 3) << 1.0, 0.0, 0.0;
//       Eigen::Matrix4d dT_B_dpy = Eigen::Matrix4d::Zero();
//       dT_B_dpy.block<1, 3>(0, 3) << 0.0, 1.0, 0.0;
//       Eigen::Matrix4d dT_B_dpz = Eigen::Matrix4d::Zero();
//       dT_B_dpz.block<1, 3>(0, 3) << 0.0, 0.0, 1.0;
//       Eigen::Matrix4d dT_B_dyaw = Eigen::Matrix4d::Zero();
//       dT_B_dyaw.block<3, 3>(0, 0) = dR_B_dyaw;
//       Eigen::Matrix4d dT_M_P_hat_dpx = dT_B_dpx * constant_term;
//       Eigen::Matrix4d dT_M_P_hat_dpy = dT_B_dpy * constant_term;
//       Eigen::Matrix4d dT_M_P_hat_dpz = dT_B_dpz * constant_term;
//       Eigen::Matrix4d dT_M_P_hat_dyaw = dT_B_dyaw * constant_term;
//       Eigen::Vector3d dnB_dpx = dT_M_P_hat_dpx.block<3, 3>(0, 0) * cc;
//       Eigen::Vector3d dnB_dpy = dT_M_P_hat_dpy.block<3, 3>(0, 0) * cc;
//       Eigen::Vector3d dnB_dpz = dT_M_P_hat_dpz.block<3, 3>(0, 0) * cc;
//       Eigen::Vector3d dnB_dyaw = dT_M_P_hat_dyaw.block<3, 3>(0, 0) * cc;
//       double dkB_dpx = dT_M_P_hat_dpx.block<3, 1>(0, 3).transpose().dot(nB) +
//                        T_M_P_B_hat.getPosition().dot(dnB_dpx);
//       double dkB_dpy = dT_M_P_hat_dpy.block<3, 1>(0, 3).transpose().dot(nB) +
//                        T_M_P_B_hat.getPosition().dot(dnB_dpy);
//       double dkB_dpz = dT_M_P_hat_dpz.block<3, 1>(0, 3).transpose().dot(nB) +
//                        T_M_P_B_hat.getPosition().dot(dnB_dpz);
//       double dkB_dyaw = dT_M_P_hat_dyaw.block<3, 1>(0, 3).transpose().dot(nB)
//       +
//                         T_M_P_B_hat.getPosition().dot(dnB_dyaw);
//       double dn_diff_dpxB = n_diff.dot(dnB_dpx);
//       double dn_diff_dpyB = n_diff.dot(dnB_dpy);
//       double dn_diff_dpzB = n_diff.dot(dnB_dpz);
//       double dn_diff_dyawB = n_diff.dot(dnB_dyaw);
//       double dk_diff_dpxB = kdiff * dkB_dpx;
//       double dk_diff_dpyB = kdiff * dkB_dpy;
//       double dk_diff_dpzB = kdiff * dkB_dpz;
//       double dk_diff_dyawB = kdiff * dkB_dyaw;
//       double * jacobianMatB = jacobians[1];
//       // for n_diff
//       jacobianMatB[0] = dn_diff_dpxB;
//       jacobianMatB[1] =0;// dn_diff_dpyB;
//       jacobianMatB[2] =0;// dn_diff_dpzB;
//       jacobianMatB[3] =0;// dn_diff_dyawB;
//       // for k_diff
//       jacobianMatB[1 * num_params + 0] = dk_diff_dpxB;
//       jacobianMatB[1 * num_params + 1] =0;// dk_diff_dpyB;
//       jacobianMatB[1 * num_params + 2] =0;// dk_diff_dpzB;
//       jacobianMatB[1 * num_params + 3] =0; //dk_diff_dyawB;
//       std::stringstream ss;
//       for (int i = 0; i < 2; ++i) {
//         for (int j = 0; j < 4; ++j) {
//           ss << jacobianMatB[i* num_params + j] << ",";
//         }
//         ss << "\n";
//       }
//       LOG(ERROR) << "jacobians[1]:\n" << ss.str();
//       if ( config_.visualize_gradients) {
//         // Transform the current point and Jacobian into the odom frame
//         const voxblox::Point odom_t_odom__point =
//             T_M_R_B_hat.getPosition().cast<float>();
//         Eigen::Vector3d odom_jacobian0(dn_diff_dpxB, dn_diff_dpyB,
//         dn_diff_dpzB); Eigen::Vector3d odom_jacobian1(dk_diff_dpxB,
//         dk_diff_dpyB, dk_diff_dpzB);
//         cost_function_visuals_.addJacobian(odom_t_odom__point,
//         odom_jacobian0.cast<float>());
//         cost_function_visuals_.addJacobian(odom_t_odom__point,
//         odom_jacobian1.cast<float>());
//       }
//     } else {
//       LOG(ERROR) << "jacobians[1] is nullptr";
//     }
//   }

//   LOG(ERROR) << "PlanesCostFunction::Evaluate(...)\n";

//   // Scale and publish the visuals, then reset them for the next iteration
//   cost_function_visuals_.scaleAndPublish(100.0);
//   cost_function_visuals_.reset();

//   return true;
// }

template <typename T>
bool PlanesCostFunction::operator()(const T* const pose_A,
                                    const T* const pose_B, T* residuals) const {
  const Eigen::Matrix<T, 4, 1> t_odom_A(pose_A[0], pose_A[1], pose_A[2],
                                        static_cast<T>(1.0));
  const Eigen::Matrix<T, 4, 1> t_odom_B(pose_B[0], pose_B[1], pose_B[2],
                                        static_cast<T>(1.0));
  const T yaw_odom_A(pose_A[3]);
  const T yaw_odom_B(pose_B[3]);
  Eigen::Matrix<T, 4, 4> T_P_R_origin_casted =
      T_P_R_origin_.getTransformationMatrix().cast<T>();
  const Eigen::Matrix<T, 3, 3> RA = rotationMatrixFromYaw(yaw_odom_A);
  Eigen::Matrix<T, 4, 4> T_M_R_A_hat = Eigen::Matrix<T, 4, 4>::Zero();
  T_M_R_A_hat.block(0, 0, 3, 3) = RA;
  T_M_R_A_hat.block(0, 3, 3, 1) = t_odom_A;
  T_M_R_A_hat(3, 3) = static_cast<T>(1.0);
  Eigen::Matrix<T, 4, 4> T_M_P_A_hat =
      T_M_R_A_hat * T_P_R_origin_casted.inverse();
  // computa nA, kA
  Eigen::Matrix<T, 3, 1> nA = T_M_P_A_hat.block(0, 2, 3, 1);
  T kA = (nA.transpose() * (T_M_P_A_hat.block(0, 3, 3, 1)))(0, 0);
  // compute the transformation ^T_M_P for destination
  Eigen::Matrix<T, 4, 4> T_P_R_destination_casted =
      T_P_R_destination_.getTransformationMatrix().cast<T>();
  const Eigen::Matrix<T, 3, 3> RB = rotationMatrixFromYaw(yaw_odom_B);
  Eigen::Matrix<T, 4, 4> T_M_R_B_hat = Eigen::Matrix<T, 4, 4>::Zero();
  T_M_R_B_hat.block(0, 0, 3, 3) = RB;
  T_M_R_B_hat.block(0, 3, 3, 1) = t_odom_B;
  T_M_R_B_hat(3, 3) = static_cast<T>(1.0);
  Eigen::Matrix<T, 4, 4> T_M_P_B_hat =
      T_M_R_B_hat * T_P_R_destination_casted.inverse();
  // compute nB, kB
  Eigen::Matrix<T, 3, 1> nB = T_M_P_B_hat.block(0, 2, 3, 1);
  T kB = (nB.transpose() * (T_M_P_B_hat.block(0, 3, 3, 1)))(0, 0);
  // compute normals residual (n1-n2)**2
  Eigen::Matrix<T, 3, 1> n_diff = nB - nA;
  residuals[0] = 0.001 * (n_diff.transpose() * n_diff)(0, 0);
  // compute offsets residual (k1-k2)**2
  T kdiff = kB - kA;
  residuals[1] = 0.001 * kdiff * kdiff;

  // constraints
  const auto T_M_PA_init = (T_M_R_origin_init_ * T_P_R_origin_.inverse());
  const auto T_M_PB_init = (T_M_R_destination_init_ * T_P_R_destination_.inverse());
  const auto nA_init_pre_casted =
      T_M_PA_init.getRotationMatrix().col(2).stableNormalized();
  const auto nB_init_pre_casted =
      T_M_PB_init.getRotationMatrix().col(2).stableNormalized();
  Eigen::Matrix<T, 3, 1> nA_init = nA_init_pre_casted.cast<T>();
  Eigen::Matrix<T, 3, 1> nB_init = nB_init_pre_casted.cast<T>();
  // Eigen::Vector3f pA_init = T_M_PA_init.getPosition();
  // Eigen::Vector3f pB_init = T_M_PB_init.getPosition();
  Eigen::Matrix<T, 3, 1> diff_positionA =
      T_M_PA_init.getPosition().cast<T>() - t_odom_A.block(3, 1, 0, 0);
  Eigen::Matrix<T, 3, 1> diff_positionB =
      T_M_PB_init.getPosition().cast<T>() - t_odom_B.block(3, 1, 0, 0);
  const auto dvecA =
      (diff_positionA - nA_init * (nA_init.transpose() * diff_positionA));
  const auto dvecB =
      (diff_positionB - nB_init * (nB_init.transpose() * diff_positionB));
  residuals[2] = 0.001 * (dvecA.transpose() * dvecA)(0, 0);
  residuals[3] = 0.001 * (dvecB.transpose() * dvecB)(0, 0);

  if (verbose_) {
    std::cout << "t_odom_A_init: " << T_M_PA_init.getPosition()(0) << ", " << T_M_PA_init.getPosition()(1) << ", " << T_M_PA_init.getPosition()(2) << "\n"
              << "t_odom_B_init: " << T_M_PB_init.getPosition()(0) << ", " << T_M_PB_init.getPosition()(1) << ", " << T_M_PB_init.getPosition()(2) << "\n"
              << "t_odom_A: " << t_odom_A(0) << ", " << t_odom_A(1) << ", "
              << t_odom_A(2) << "\n"
              << "t_odom_B: " << t_odom_B(0) << ", " << t_odom_B(1) << ", "
              << t_odom_B(2) << "\n"
              << "RA:\n" << RA << "\n"
              << "RB:\n" << RB << "\n"
              << "nA_init:\n" << nA_init << "\n"
              << "nB_init:\n" << nB_init << "\n"
              << "nA:\n" << nA << "\n"
              << "nB:\n" << nA << "\n"
              << "residuals[0]=" << residuals[0] << "\n"
              << "T_M_P_A_hat:\n" << T_M_P_A_hat << "\n"
              << "T_M_P_B_hat:\n" << T_M_P_B_hat << "\n"
              << "kA=" << kA << "\n"
              << "kB=" << kB << "\n"
              << "residuals[1]=" << residuals[1] << "\n"
              << "residuals[2]=" << residuals[2] << "\n"
              << "residuals[3]=" << residuals[3] << "\n";
  }
  return true;

  // CHECK_NEAR(yaw_odom_A, static_cast<T>(0.0), static_cast<T>(M_PI));
  // CHECK_NEAR(yaw_odom_B, static_cast<T>(0.0), static_cast<T>(M_PI));
  // for A
  // LOG(ERROR) << "COMPUTED TILL A";
  // Eigen::Matrix<T, 3, 3> RA = rotationMatrixFromYaw(yaw_odom_A);
  // Eigen::Matrix<T, 4, 4> T_M_R_A_hat = Eigen::Matrix<T, 4, 4>::Zero();
  // T_M_R_A_hat.block(0, 0, 3, 3) = RA;
  // T_M_R_A_hat.block(0, 3, 3, 1) = t_odom_A;
  // T_M_R_A_hat(3, 3) = static_cast<T>(1.0);
  // Eigen::Matrix<T, 4, 4> T_M_P_A_hat =
  //     T_M_R_A_hat *
  // T_P_R_origin_.inverse().getTransformationMatrix().cast<T>();
  // Eigen::Matrix<T, 3, 1> nA = Eigen::Matrix<T, 3,
  // 1>::Zero();//std::move(T_M_P_A_hat.block(0, 2, 3, 1)); LOG(ERROR) <<
  // "COMPUTED TILL B"; T kA = nA.transpose() * (T_M_P_A_hat.block(0,3,3,1)); T
  // kA = nA.data()[0] * T_M_P_A_hat.data()[3 * 4 + 0] +
  //        nA.data()[1] * T_M_P_A_hat.data()[3 * 4 + 1] +
  //        nA.data()[2] * T_M_P_A_hat.data()[3 * 4 + 2];
  // T kA = static_cast<T> (0);
  // for B
  // Eigen::Matrix<T, 3, 3> RB = rotationMatrixFromYaw(yaw_odom_B);
  // Eigen::Matrix<T, 4, 4> T_M_R_B_hat = Eigen::Matrix<T, 4, 4>::Zero();
  // T_M_R_B_hat.block(0, 0, 3, 3) = RB;
  // T_M_R_B_hat.block(0, 3, 3, 1) = t_odom_B;
  // T_M_R_B_hat(3, 3) = static_cast<T>(1.0);
  // Eigen::Matrix<T, 4, 4> T_M_P_B_hat =
  //     T_M_R_B_hat *
  //     T_P_R_destination_.inverse().getTransformationMatrix().cast<T>();
  // Eigen::Matrix<T, 3, 1> nB = Eigen::Matrix<T,
  // 3,1>::Zero();//std::move(T_M_P_B_hat.block(0, 2, 3, 1)); T kB =
  // nB.data()[0] * T_M_P_B_hat.data()[3 * 4 + 0] +
  //        nB.data()[1] * T_M_P_B_hat.data()[3 * 4 + 1] +
  //        nB.data()[2] * T_M_P_B_hat.data()[3 * 4 + 2];
  // T kB = static_cast<T>(0);
  // Eigen::Map<Eigen::Matrix<T, 2, 1>> residuals_map(residuals);
  // LOG(ERROR) << "COMPUTED TILL C";
  // residuals[0] = (nA - nB).transpose().dot(nA - nB);
  // residuals[1] = (kA - kB) * (kA - kB);
  // LOG(ERROR) << "COMPUTED TILL D";
  // return true;
}

ceres::CostFunction* PlanesCostFunction::Create(
    const Transformation& T_M_R_origin, const PlaneType::ConstPtr origin_plane,
    const Transformation& T_M_R_destination,
    const PlaneType::ConstPtr destination_plane,
    const Constraint::InformationMatrix& sqrt_information_matrix,
    const PlanesCostFunction::Config& planes_cost_config) {
  return (new ceres::AutoDiffCostFunction<PlanesCostFunction, 4, 4, 4>(
      new PlanesCostFunction(T_M_R_origin, origin_plane, T_M_R_destination,
                             destination_plane, sqrt_information_matrix,
                             planes_cost_config)));
  // return new PlanesCostFunction(T_M_R_origin, origin_plane,
  // T_M_R_destination,
  //                               destination_plane, sqrt_information_matrix,
  //                               planes_cost_config);
}

}  // namespace voxgraph