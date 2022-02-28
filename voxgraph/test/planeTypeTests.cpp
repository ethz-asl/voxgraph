#include <voxgraph/frontend/plane_collection/plane_type.h>
#include <voxgraph/frontend/plane_collection/bounding_box_extended.h>

#include <cstdlib>
#include <fstream>
#include <queue>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Eigen>
#include <experimental/filesystem>
#include <gtest/gtest.h>

namespace voxgraph {
namespace test {

typedef Eigen::Hyperplane<float, 3> EigenPlane;

PlaneType createAPlane(Point p=Point(0,0,0), Point n=Point(0,0,1)) {
  return PlaneType(EigenPlane(n,p),0);
}

TEST(planeCreation, planeTypeTests) {
  Point p(0,10,0), n(0,0,1);
  PlaneType pA(n,p,0);

  CHECK_EQ(pA.getPointInit(), p);
  CHECK_EQ(pA.getPlaneNormal(), n);
}

TEST(plane, planeTypeTests) {
  
}

TEST(planeFromFunction, planeTypeTests) {
  std::vector<PlaneType> planes;
  for (int i = 0; i < 10; ++i) {
    planes.push_back(createAPlane(Point(i,i,i)));
  }
  for (int i = 1; i < 10; ++i) {
    CHECK_NE(planes[i].getPlaneID(), planes[i-1].getPlaneID());
  }
}

TEST(fromToMsg, planeTypeTests) {
  const Point n(0,0.5,0.5);
  const Point p(100,70,123);
  LOG(INFO) << "n.stableNormalized() =\n" << n.stableNormalized();
  const PlaneType pA = PlaneType(n.stableNormalized(),p, 0);
  CHECK_EQ(pA.getPointInit(), p);
  const PlaneType pB = PlaneType::fromMsg(pA.toPlaneTypeMsg());
  CHECK_EQ(pA.getPointInit(), pB.getPointInit());
  CHECK_EQ(pA.getPlaneNormal(), pB.getPlaneNormal());
  CHECK_EQ(pA.getNumPoints(), pB.getNumPoints());
}

}  // namespace test
}  // namespace voxgraph

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}