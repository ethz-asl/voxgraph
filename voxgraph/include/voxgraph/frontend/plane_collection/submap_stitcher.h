#ifndef VOXGRAPH_FRONTEND_PLANE_COLLECTION_SUBMAP_STITCHER_H_
#define VOXGRAPH_FRONTEND_PLANE_COLLECTION_SUBMAP_STITCHER_H_

#include <map>
#include <utility>
#include <vector>

#include "voxgraph/frontend/plane_collection/plane_type.h"
#include "voxgraph/frontend/submap_collection/voxgraph_submap.h"

namespace voxgraph {

class SubmapStitcher {
 public:
  typedef const SubmapStitcher* ConstPtr;
  struct Config {
    float threshold_dist = 1.5;
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

 private:
  std::shared_ptr<std::map<int, classToPlanesType>>
      submap_id_to_class_to_planes_;
  std::map<int, std::shared_ptr<PlaneType>> all_planes_;
  Config config_;
};

}  // namespace voxgraph

#endif  // VOXGRAPH_FRONTEND_PLANE_COLLECTION_SUBMAP_STITCHER_H_
