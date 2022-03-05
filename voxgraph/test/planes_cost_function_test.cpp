#include <cstdlib>
#include <fstream>
#include <queue>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Eigen>
#include <ceres/ceres.h>
#include <eigen_stl_containers/eigen_stl_vector_container.h>
#include <experimental/filesystem>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <tf2_ros/transform_broadcaster.h>
#include <voxgraph/backend/constraint/cost_functions/planes_cost_function.h>
#include <voxgraph/frontend/plane_collection/bounding_box_extended.h>
#include <voxgraph/frontend/plane_collection/plane_type.h>

namespace voxgraph {
namespace test {

class PlanesConstFunctionTests {
 public:
  typedef std::list<ceres::Solver::Summary> SolverSummaryList;
  PlanesConstFunctionTests() = default;
  explicit PlanesConstFunctionTests(
      const float hz, const std::vector<Point>& plane_points,
      const std::vector<Point>& normal_points,
      const std::vector<Transformation::Vector6>& tf6s,
      const std::vector<rviz_visual_tools::colors>& plane_colors,
      const std::vector<std::string>& frame_names)
      : nh_(""),
        loop_rate_(hz),
        visual_tools_(new rviz_visual_tools::RvizVisualTools(
            "world", "/rviz_visual_markers")),
        sqrt_information_matrix_(Eigen::Matrix4d::Identity()),
        plane_colors_(plane_colors),
        frame_names_(frame_names) {
    visual_tools_->deleteAllMarkers();
    createTransformations(tf6s);
    createPlaneExamples(plane_points, normal_points);

    int idx = 0;
    for (const auto& plane_ptr : planes_) {
      drawPlane(*plane_ptr, plane_colors[idx++], 10);
    }

    initCeresSolver();
  }

  void createTransformations(const std::vector<Transformation::Vector6>& tf6s) {
    // tB << 1.0, 2.0, 1.0, 0, 0, 1.0;
    int idx = 0;
    for (const auto& tf6 : tf6s) {
      const Transformation tfexp = Transformation::exp(tf6);
      T_M_R__init_.push_back(tfexp);
      T_M_R__4D.push_back(std::make_shared<Pose4D>(tfexp));
      std::cout << "T_M_R__init_[" << idx << " ]:\n" << tfexp << "\n";
      idx++;
    }
    // ReferenceFrameNode::Config cfg;
    // cfg.reference_frame_id = 0;
    // nodeCollection_.addReferenceFrameNode();
    visualizeTransformations();
  }

  void visualizeTransformations() {
    for (size_t i = 0; i < T_M_R__init_.size(); ++i) {
      visualizeTransform(T_M_R__init_[i], frame_names_[i] + "init", 100 + i);
      visualizeTransform(T_M_R__4D[i]->operator voxgraph::Transformation(),
                         frame_names_[i], 100 + i);
    }
    // visual_tools_->publishText(rviz_visual_tools::convertPointToPose(T_M_R__init_[0].getPosition().cast<double>()),
    // "T_M_R__A", rviz_visual_tools::MAGENTA, Eigen::Vector3d(10,20,10));
    // visual_tools_->publishText(rviz_visual_tools::convertPointToPose(T_M_R__init_[1].getPosition().cast<double>()),
    // "T_M_R__B", rviz_visual_tools::MAGENTA, Eigen::Vector3d(10,20,10));
  }

  void visualizeTransform(const Transformation& t,
                          const std::string& child_frame_id, const int seq) {
    geometry_msgs::TransformStamped msg;
    tf::transformKindrToMsg(t.cast<double>(), &msg.transform);
    msg.header.frame_id = "world";
    msg.header.seq = seq;
    msg.child_frame_id = child_frame_id;
    msg.header.stamp = ros::Time::now();
    br_.sendTransform(msg);
  }

  void drawPlane(const PlaneType& plane, const rviz_visual_tools::colors& color,
                 int id = -1, const bool delete_prev = true) {
    if (id == -1) {
      id = plane.getPlaneID();
    }
    auto msg = plane.getVisualizationMsg();
    msg.id = 2 * id;
    auto normal_msg = plane.getNormalVisualizationMsg();
    // normal_msg.id = 2 * plane.getPlaneID() + 1;
    visual_tools_->publishArrow(normal_msg.points[0], normal_msg.points[1],
                                rviz_visual_tools::RED,
                                rviz_visual_tools::MEDIUM, 2 * id + 1);
    // visual_tools_->publishWireframeCuboid(pose, 0.5,2.0,1.0
    //                              , rviz_visual_tools::BLUE, "Plane", 2 *
    //                              plane.getPlaneID());
    geometry_msgs::Pose pose_msg;
    tf::poseKindrToMsg(plane.getPlaneTransformation().cast<double>(),
                       &pose_msg);
    Point dwh = Point(1.0, 2.0, 0.1);
    visual_tools_->publishCuboid(pose_msg, dwh(0), dwh(1), dwh(2), color);
  }

  void createPlaneExamples(const std::vector<Point>& points,
                           const std::vector<Point>& normals) {
    const size_t s_s = points.size();
    for (size_t idx = 0; idx < s_s; ++idx) {
      const auto new_plane = std::make_shared<PlaneType>(
          normals[idx].stableNormalized(), points[idx], 2 * idx);
      planes_.push_back(new_plane);
      T_P_R__.push_back(new_plane->getPlaneTransformation().inverse() *
                        T_M_R__init_[idx]);
      planes__paths_.push_back(EigenSTL::vector_Isometry3d{
          Eigen::Isometry3d(new_plane->getPlaneTransformation()
                                .getTransformationMatrix()
                                .cast<double>())});
    }
  }

  void initCeresSolver() {
    problem_ptr_ = std::make_shared<ceres::Problem>(problem_options_);
    problem_ptr_->AddParameterBlock(T_M_R__4D[0]->optimizationVectorData(),
                                    T_M_R__4D[0]->optimizationVectorSize());
    problem_ptr_->AddParameterBlock(T_M_R__4D[1]->optimizationVectorData(),
                                    T_M_R__4D[1]->optimizationVectorSize());
    // Set the local parameterization s.t. yaw stays normalized
    for (size_t idx = 0; idx < T_M_R__4D.size(); ++idx) {
      problem_ptr_->SetParameterization(
          T_M_R__4D[idx]->optimizationVectorData(),
          nodeCollection_.getLocalParameterization());
    }
    problem_ptr_->SetParameterBlockConstant(
        T_M_R__4D[0]->optimizationVectorData());
    PlanesCostFunction::Config cfg;
    const size_t s_s = T_M_R__4D.size()-1;
    ceres::LossFunction* loss_function = nullptr;
    for(size_t idx = 0; idx < s_s; ++idx) {
      ceres::CostFunction* cost_function = PlanesCostFunction::Create(
          T_M_R__init_[idx], planes_[idx].get(), T_M_R__init_[idx+1], planes_[idx+1].get(),
          sqrt_information_matrix_, cfg);
      residual_block_id_ = problem_ptr_->AddResidualBlock(
          cost_function, loss_function, T_M_R__4D[idx]->optimizationVectorData(),
          T_M_R__4D[idx+1]->optimizationVectorData());
    }

  }

  void optimize(const bool delete_prev = true) {
    // do optimization work
    // TransformationD::Vector6 tA6D =
    // TransformationD::log(T_M_R__init_[0].cast<double>());
    // TransformationD::Vector6 tB6D =
    // TransformationD::log(T_M_R__init_[1].cast<double>());

    // auto pdata_A = T_M_R__4D[0].optimizationVectorData();
    // auto pdata_B = T_M_R__4D[1].optimizationVectorData();
    // pdata_A[0] = tA6D[0];
    // pdata_A[1] = tA6D[1];
    // pdata_A[2] = tA6D[2];
    // pdata_A[3] = tA6D[5];
    // pdata_B[0] = tB6D[0];
    // pdata_B[1] = tB6D[1];
    // pdata_B[2] = tB6D[2];
    // pdata_B[3] = tB6D[5];

    ceres::Solver::Summary summary;
    ceres::Solve(solver_options_, problem_ptr_.get(), &summary);
    std::cout << "Optimized:\n"
              << "- parameter blocks original " << summary.num_parameter_blocks
              << " -> reduced " << summary.num_parameter_blocks_reduced << "\n"
              << "- cost initial " << summary.initial_cost << " -> final "
              << summary.final_cost << "\n"
              << "- iterations successful " << summary.num_successful_steps
              << " -> unsuccessful " << summary.num_unsuccessful_steps << "\n"
              << "- total time " << summary.total_time_in_seconds << "\n"
              << "- termination "
              << ceres::TerminationTypeToString(summary.termination_type)
              << std::endl;
    // show results
    Transformation T_M_R_A_hat =
        T_M_R__4D[0]->operator voxgraph::Transformation();
    Transformation T_M_R_B_hat =
        T_M_R__4D[1]->operator voxgraph::Transformation();
    planes_[0]->transformPlane(T_P_R__[0] * T_M_R__init_[0].inverse() *
                               T_M_R_A_hat * T_P_R__[0].inverse());
    planes_[1]->transformPlane(T_P_R__[1] * T_M_R__init_[1].inverse() *
                               T_M_R_B_hat * T_P_R__[1].inverse());
    if (delete_prev) {
      visual_tools_->deleteAllMarkers();
    }
    if (summary.final_cost != summary.initial_cost) {
      planes__paths_[0].push_back(
          Eigen::Isometry3d(planes_[0]
                                ->getPlaneTransformation()
                                .getTransformationMatrix()
                                .cast<double>()));
      planes__paths_[1].push_back(
          Eigen::Isometry3d(planes_[1]
                                ->getPlaneTransformation()
                                .getTransformationMatrix()
                                .cast<double>()));
    }
    visualizeTransformations();
    drawPlane(*planes_[0], rviz_visual_tools::BLUE);
    drawPlane(*planes_[1], rviz_visual_tools::LIME_GREEN);
    visual_tools_->publishAxisPath(planes__paths_[0],
                                   rviz_visual_tools::MEDIUM);
    visual_tools_->publishAxisPath(planes__paths_[1],
                                   rviz_visual_tools::MEDIUM);
    visual_tools_->trigger();
  }

  void startMainLoop() {
    while (ros::ok()) {
      std::cout << "main_loop" << std::endl;
      ros::spinOnce();
      optimize(false);
      loop_rate_.sleep();
    }
  }

 private:
  std::vector<Transformation> T_P_R__;
  std::vector<Transformation> T_M_R__init_;
  std::vector<std::shared_ptr<Pose4D>> T_M_R__4D;
  std::vector<std::shared_ptr<PlaneType>> planes_;
  std::vector<EigenSTL::vector_Isometry3d> planes__paths_;
  std::vector<std::string> frame_names_;
  ros::NodeHandle nh_;
  ros::Rate loop_rate_;
  ceres::Solver::Options solver_options_;
  ceres::Problem::Options problem_options_;
  std::shared_ptr<ceres::Problem> problem_ptr_;
  SolverSummaryList solver_summaries_;
  ceres::ResidualBlockId residual_block_id_;
  NodeCollection nodeCollection_;
  Constraint::InformationMatrix sqrt_information_matrix_;
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
  tf2_ros::TransformBroadcaster br_;
  std::vector<rviz_visual_tools::colors> plane_colors_;
};

void test_2_vertical_planes_with_yaw() {
  const float hertz = 1.0 / 4.0;
  std::vector<rviz_visual_tools::colors> plane_colors{
      rviz_visual_tools::BLUE, rviz_visual_tools::LIME_GREEN};
  std::vector<Transformation::Vector6> tf6s{
      (Transformation::Vector6() << 1.0, 2.0, 1.0, 0, 0, 1.0).finished(),
      (Transformation::Vector6() << 2.0, 3.0, 0.5, 0, 0, -1.0).finished()};
  std::vector<Point> plane_points{Point(10.0, 17.0, 1.0), Point(5, 3, -0.8)};
  std::vector<Point> normal_points{Point(1.0, 0.0, 0.0), Point(0.8, 0.2, 0.0)};
  std::vector<std::string> frame_names{"A", "B"};
  PlanesConstFunctionTests tt(hertz, plane_points, normal_points, tf6s,
                              plane_colors, frame_names);
  tt.startMainLoop();
}

void test_2_horizontal_planes_with_pitch_roll() {
  const float hertz = 1.0 / 4.0;
  std::vector<rviz_visual_tools::colors> plane_colors{
      rviz_visual_tools::BLUE, rviz_visual_tools::LIME_GREEN};
  std::vector<Transformation::Vector6> tf6s{
      (Transformation::Vector6() << 1.0, 2.0, 1.0, 0, 0, 1.0).finished(),
      (Transformation::Vector6() << 2.0, 3.0, 0.5, 0, 0, -1.0).finished()};
  std::vector<Point> plane_points{Point(10.0, 17.0, 1.0), Point(5, 3, -0.8)};
  std::vector<Point> normal_points{Point(0.0, 0.0, 1.0), Point(0.2, 0.0, 0.8)};
  std::vector<std::string> frame_names{"A", "B"};
  PlanesConstFunctionTests tt(hertz, plane_points, normal_points, tf6s,
                              plane_colors, frame_names);
  tt.startMainLoop();
}

}  // namespace test
}  // namespace voxgraph

// run in separate shells:
// - roscore
// - rviz
// before launching this test with command:
// devel/lib/voxgraph/planes_cost_function-test
int main(int argc, char** argv) {
  // testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);
  ros::init(argc, argv, "planesCostFunctionTests");
  voxgraph::test::test_2_horizontal_planes_with_pitch_roll();
  return 0;
}