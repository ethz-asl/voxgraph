#include "voxgraph/frontend/plane_collection/submap_stitcher.h"

#include <map>
#include <utility>
#include <vector>

#include "voxgraph/frontend/plane_collection/plane_type.h"
#include "voxgraph/frontend/submap_collection/voxgraph_submap.h"

namespace voxgraph {

SubmapStitcher::SubmapStitcher() {
  submap_id_to_class_to_planes_ =
      std::make_shared<std::map<int, classToPlanesType>>();
}

void SubmapStitcher::addSubmapPlanes(const int submap_id,
                                     const classToPlanesType& c_t_p) {
  // add all planes to plane id --> plane map
  for (const auto& pair : c_t_p) {
    for (const auto& plane : pair.second) {
      all_planes_[plane.getPlaneID()] = std::make_shared<PlaneType>(plane);
    }
  }
  // keep all planes in a submap (id,class) --> planes double map
  submap_id_to_class_to_planes_->insert({submap_id, c_t_p});
}

void SubmapStitcher::addSubmapPlanes(const int submap_id,
                                     const classToPlanesType&& c_t_p) {
  // add all planes to plane id --> plane map
  for (const auto& pair : c_t_p) {
    for (const auto& plane : pair.second) {
      all_planes_[plane.getPlaneID()] = std::make_shared<PlaneType>(plane);
    }
  }
  // keep all planes in a submap (id,class) --> planes double map
  (*submap_id_to_class_to_planes_)[submap_id] = std::move(c_t_p);
}

void SubmapStitcher::matchArrayOfPlanes(const classToPlanesType& planeSeriesA,
                                        const classToPlanesType& planeSeriesB,
                                        std::map<int, int>* matched_planes) {
  CHECK_NOTNULL(matched_planes);
  // greedy bibartite matching
  // normally we need the Hungarian algorithm
  for (const auto& pairA : planeSeriesA) {
    if (planeSeriesB.find(pairA.first) != planeSeriesB.end()) {
      // get the closest planes for each of the maps
      for (const PlaneType& planeA : pairA.second) {
        float dist2_min = +INFINITY;
        int closest_plane_id = -1;
        for (const PlaneType& planeB : planeSeriesB.at(pairA.first)) {
          float dist2_current = planeB.distSquared(planeA);
          if (dist2_current < dist2_min) {
            dist2_min = dist2_current;
            closest_plane_id = planeB.getPlaneID();
          }
        }
        if (dist2_min < config_.threshold_dist) {
          matched_planes->insert({planeA.getPlaneID(), closest_plane_id});
        }
      }
    }
  }
}

void SubmapStitcher::findAllPlaneMatchesForSubmap(
    const VoxgraphSubmap& s,
    const std::vector<VoxgraphSubmap::ConstPtr>& all_submaps,
    std::map<int, int>* matched_planes, std::vector<int>* submap_ids) {
  CHECK_NOTNULL(matched_planes);
  CHECK(submap_ids);
  // you should have added the submap `s` first by calling addSubmapPlanes
  std::vector<int> neighboor_ids;
  int n = findNeighboorsToSubmap(s, all_submaps, &neighboor_ids);
  LOG(ERROR) << "Number of submap neighboors:" << n << " out of " << all_submaps.size() << " submaps";
  size_t s_idx = 0u;
  for (const int ns : neighboor_ids) {
    matchArrayOfPlanes(submap_id_to_class_to_planes_->at(s.getID()),
                       submap_id_to_class_to_planes_->at(ns), matched_planes);
    const size_t num_all_matched_planes = matched_planes->size();
    // add respective submap id to newly added planes
    while (s_idx < num_all_matched_planes) {
      submap_ids->push_back(ns);
      ++s_idx;
    }
  }
}

int SubmapStitcher::findNeighboorsToSubmap(
    const VoxgraphSubmap& s,
    const std::vector<VoxgraphSubmap::ConstPtr>& all_submaps,
    std::vector<int>* neighboor_ids) {
  for (const auto& sptr : all_submaps) {
    if (sptr->getID() == s.getID()) {
      continue;
    }
    // if (s.overlapsWith(*sptr)) {
      neighboor_ids->push_back(sptr->getID());
    // }
  }
  return neighboor_ids->size();
}

}  // namespace voxgraph
