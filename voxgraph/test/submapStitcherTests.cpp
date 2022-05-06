#include <cstdlib>
#include <fstream>
#include <functional>
#include <queue>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Eigen>
#include <experimental/filesystem>
#include <gtest/gtest.h>
#include <voxgraph/frontend/plane_collection/submap_stitcher.h>

namespace voxgraph {
namespace test {

typedef Eigen::Hyperplane<float, 3> EigenPlane;

PlaneType createAPlane(const Point p = Point(0, 0, 0),
                       const Point n = Point(0, 0, 1)) {
  return PlaneType(EigenPlane(n, p), 0);
}

void createPlanes(std::vector<PlaneType>* planes, const std::vector<Point>& ps,
                  const std::vector<Point>& ns) {
  size_t num = ps.size();
  EXPECT_EQ(ps.size(), ns.size());
  for (size_t i = 0; i < num; ++i) {
    planes->emplace_back(ps.at(i), ns.at(i), 0);
  }
}

void createPlanes(std::vector<PlaneType>* planes,
                  std::function<std::pair<Point, Point>(int)>&& f,
                  const size_t num) {
  for (size_t i = 0; i < num; ++i) {
    std::pair<Point, Point> p_n = f(i);
    Point p = p_n.first;
    Point n = p_n.second;
    planes->emplace_back(n, p, 0);
  }
}

bool checkIfPlanesShouldMatch(const PlaneType& planeA,
                              const PlaneType& planeB) {
  const auto dist = planeA.distSquared(planeB);
  if (planeA.getPlaneNormal().dot(planeB.getPlaneNormal()) < 0.0) {
    LOG(INFO) << "planeA.getPlaneNormal().dot(planeB.getPlaneNormal()) < 0.0";
    return false;
  }
  if ((planeA.getPointInit() - planeB.getPointInit()).squaredNorm() > 2.0) {
    LOG(INFO) << "(planeA.getPointInit() - "
                 "planeB.getPointInit()).squaredNorm() > 2.0";
    return false;
  }

  return true;
}

TEST(create, submapStitcherTests) { SubmapStitcher ss(); }

TEST(addSubmapPlanes, submapStitcherTests) {
  SubmapStitcher ss();
  classToPlanesType cctp;
  cctp[0] = std::vector<PlaneType>{};
  createPlanes(
      &cctp[0],
      [](int i) {
        return std::make_pair(Point(0, 0, 10.0 * i), Point(0, 0, 1.0));
      },
      1000);
  // ss.addSubmapPlanes(0, cctp);
}

TEST(oppositePlanes, submapStitcherTests) { SubmapStitcher ss(); }

}  // namespace test
}  // namespace voxgraph

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}