#ifndef VOXGRAPH_FRONTEND_PLANE_COLLECTION_SUBMAP_STITCHER_H_
#define VOXGRAPH_FRONTEND_PLANE_COLLECTION_SUBMAP_STITCHER_H_

#include <map>
#include <utility>
#include <vector>

#include <rviz_visual_tools/rviz_visual_tools.h>
#include "voxgraph/frontend/plane_collection/plane_type.h"
#include "voxgraph/frontend/submap_collection/voxgraph_submap.h"

namespace voxgraph {

/**
 * @brief This class aims in finding if two planes are close enough to create a constraint
 * based on them.
 */
class SubmapStitcher {
 public:
  typedef const SubmapStitcher* ConstPtr;
  struct Config {
    // threshold of the anglie between the planes
    float threshold_normal_radians = 10.0 * M_PI / 180.0f;
    // threshold of the distance over their normals between the planes
    float threshold_normal_distance = 0.50;
    // threshold of the distance over their normals between the planes,
    // if their normal vectors are in reverse order
    float threshold_normal_reversed_distance = 0.15;
  };
  SubmapStitcher();
  void addSubmapPlanes(const int submap_id, const classToPlanesType& c_t_p);
  void addSubmapPlanes(const int submap_id, const classToPlanesType&& c_t_p);
  void matchArrayOfPlanes(const classToPlanesType& planeSeriesA,
                          const classToPlanesType& planeSeriesB,
                          std::map<int, int>* matched_planes);
  void findAllPlaneMatchesForSubmap(
      const VoxgraphSubmap& s,
      const std::vector<VoxgraphSubmap::ConstPtr>& all_submaps,
      std::map<int, int>* matched_planes, std::vector<int>* submap_ids);
  int findNeighboorsToSubmap(
      const VoxgraphSubmap& s,
      const std::vector<VoxgraphSubmap::ConstPtr>& all_submaps,
      std::vector<int>* neighboor_ids);
  // bool stitch(const Submap& SubmapA, Submap* submapB);
  std::map<int, std::shared_ptr<PlaneType>> getAllPlanes() const {
    return all_planes_;
  }
  void visualizeVector(const Eigen::Vector3f vec,
                       const Eigen::Vector3f point) const;

 private:
  std::shared_ptr<std::map<int, classToPlanesType>>
      submap_id_to_class_to_planes_;
  std::map<int, std::shared_ptr<PlaneType>> all_planes_;
  Config config_;
  ros::Publisher vec_pub_;
  ros::NodeHandle nh_;
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
};

}  // namespace voxgraph

#endif  // VOXGRAPH_FRONTEND_PLANE_COLLECTION_SUBMAP_STITCHER_H_
