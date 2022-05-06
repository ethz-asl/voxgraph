#include "voxgraph/backend/constraint/cost_functions/planes_cost_function.h"

#include <utility>

#include <kindr/minimal/quat-transformation.h>
#include <kindr/minimal/rotation-quaternion.h>

#include "voxgraph/tools/tf_helper.h"

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
      verbose_(false) {
  CHECK(origin_plane);
  CHECK(destination_plane);
  CHECK_NE(origin_plane->getPlaneID(), destination_plane_->getPlaneID());
  T_M_P_origin_ = origin_plane->getPlaneTransformation();
  T_P_R_origin_ = T_M_P_origin_.inverse() * T_M_R_origin_init_;
  T_M_P_destination_ = destination_plane->getPlaneTransformation();
  T_P_R_destination_ = T_M_P_destination_.inverse() * T_M_R_destination_init_;
  T_origin_desination_ = T_M_R_origin.inverse() * T_M_R_destination;
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

template <typename T>
bool PlanesCostFunction::operator()(const T* const pose_A,
                                    const T* const pose_B, T* residuals) const {
  typedef Eigen::Matrix<T, 3, 1> Vector3T;
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
                         T_M_R_A_hat.block(0, 0, 3, 3) = RA;        // NOLINT
                         T_M_R_A_hat.block(0, 3, 3, 1) = t_odom_A;  // NOLINT
                         T_M_R_A_hat(3, 3) = static_cast<T>(1.0);   // NOLINT
  Eigen::Matrix<T, 4, 4> T_M_P_A_hat =
      T_M_R_A_hat * T_P_R_origin_casted.inverse();
  // computa nA, kA
  Eigen::Matrix<T, 3, 1> nA = T_M_P_A_hat.block(0, 2, 3, 1);
  Eigen::Matrix<T, 3, 1> pA = T_M_P_A_hat.block(0, 3, 3, 1);
  // compute the transformation ^T_M_P for destination
  Eigen::Matrix<T, 4, 4> T_P_R_destination_casted =
      T_P_R_destination_.getTransformationMatrix().cast<T>();
  const Eigen::Matrix<T, 3, 3> RB = rotationMatrixFromYaw(yaw_odom_B);
  Eigen::Matrix<T, 4, 4> T_M_R_B_hat = Eigen::Matrix<T, 4, 4>::Zero();
                         T_M_R_B_hat.block(0, 0, 3, 3) = RB;        // NOLINT
                         T_M_R_B_hat.block(0, 3, 3, 1) = t_odom_B;  // NOLINT
                         T_M_R_B_hat(3, 3) = static_cast<T>(1.0);   // NOLINT
  Eigen::Matrix<T, 4, 4> T_M_P_B_hat =
      T_M_R_B_hat * T_P_R_destination_casted.inverse();
  // compute nB, kB
  Eigen::Matrix<T, 3, 1> nB = T_M_P_B_hat.block(0, 2, 3, 1);
  Eigen::Matrix<T, 3, 1> pB = T_M_P_B_hat.block(0, 3, 3, 1);
  // transformations
  Eigen::Matrix<T, 3, 1> dpApB = pB - pA;
  Eigen::Matrix<T, 3, 1> dpBpA = pA - pB;
  Eigen::Matrix<T, 3, 1> vec_dist1 = nA.cwiseProduct(dpApB);
  Eigen::Matrix<T, 3, 1> vec_dist2 = nB.cwiseProduct(dpBpA);

  T cosineAB = (nA.transpose() * nB)(0, 0);
  T sinAB_squared = static_cast<T>(1.0) - (cosineAB * cosineAB);

  // if (sinAB_squared < 0.2) {
    residuals[0] = static_cast<T>(sqrt_information_matrix_(0, 0)) *
                   (vec_dist1(0) * vec_dist1(0) + vec_dist2(0) * vec_dist2(0));
    residuals[1] = static_cast<T>(sqrt_information_matrix_(1, 1)) *
                   (vec_dist1(1) * vec_dist1(1) + vec_dist2(1) * vec_dist2(1));
    residuals[2] = static_cast<T>(sqrt_information_matrix_(2, 2)) *
                   (vec_dist1(2) * vec_dist1(2) + vec_dist2(2) * vec_dist2(2));
  // } else {
  //   residuals[0] = static_cast<T>(sqrt_information_matrix_(0, 0));
  //   residuals[1] = static_cast<T>(sqrt_information_matrix_(1, 1));
  //   residuals[2] = static_cast<T>(sqrt_information_matrix_(2, 2));
  // }
  residuals[3] = static_cast<T>(sqrt_information_matrix_(3, 3)) * sinAB_squared;
  if (verbose_) {
    Eigen::Matrix<T, 4, 4> T_PA_PB = T_M_P_A_hat.inverse() * T_M_P_B_hat;
    std::cout << "t_odom_A_init: " << T_M_R_origin_init_.getPosition()(0)
              << ", " << T_M_R_origin_init_.getPosition()(1) << ", "
              << T_M_R_origin_init_.getPosition()(2) << "\n"
              << "t_odom_B_init: " << T_M_R_destination_init_.getPosition()(0)
              << ", " << T_M_R_destination_init_.getPosition()(1) << ", "
              << T_M_R_destination_init_.getPosition()(2) << "\n"
              << "t_odom_A: " << t_odom_A(0) << ", " << t_odom_A(1) << ", "
              << t_odom_A(2) << "\n"
              << "t_odom_B: " << t_odom_B(0) << ", " << t_odom_B(1) << ", "
              << t_odom_B(2) << "\n"
              << "RA:\n"
              << RA << "\n"
              << "RB:\n"
              << RB << "\n"
              << "nA:\n"
              << nA << "\n"
              << "nB:\n"
              << nB << "\n"
              << "pA:\n"
              << pA << "\n"
              << "pB:\n"
              << pB << "\n"
              << "T_M_P_A_hat:\n"
              << T_M_P_A_hat << "\n"
              << "T_M_P_B_hat:\n"
              << T_M_P_B_hat << "\n"
              << "T_PA_PB.position:\n"
              << T_PA_PB.block(0, 3, 3, 1) << "\n"
              << "residuals[0]=" << residuals[0] << "\n"
              << "residuals[1]=" << residuals[1] << "\n"
              << "residuals[2]=" << residuals[2] << "\n"
              << "residuals[3]=" << residuals[3] << "\n";
  }
  return true;
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
}

}  // namespace voxgraph