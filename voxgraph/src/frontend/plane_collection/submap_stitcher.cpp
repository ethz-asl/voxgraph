#include "voxgraph/frontend/plane_collection/submap_stitcher.h"

#include <map>
#include <utility>
#include <vector>
#include <cmath>

#include <rviz_visual_tools/rviz_visual_tools.h>

#include "voxgraph/frontend/plane_collection/plane_type.h"
#include "voxgraph/frontend/submap_collection/voxgraph_submap.h"

namespace voxgraph {

SubmapStitcher::SubmapStitcher():nh_("")
  {
  visual_tools_ = std::make_shared<rviz_visual_tools::RvizVisualTools>("world", "/voxgraph_mapper/submap_stitcher", nh_);
  submap_id_to_class_to_planes_ =
      std::make_shared<std::map<int, classToPlanesType>>();
  // vec_pub_ = nh_.advertise<visualization_msgs::Marker>(
  //     "/voxgraph_mapper/cost_function_vecs", 100);
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
    const int class_id = pairA.first;
    if (planeSeriesB.find(class_id) != planeSeriesB.end()) {
      // get the closest planes for each of the maps
      for (const PlaneType& planeA : pairA.second) {
        for (const PlaneType& planeB : planeSeriesB.at(class_id)) {
          float normals_cosine = planeA.getPlaneNormal().dot(planeB.getPlaneNormal());
          float distance_along_normalA = planeA.dist(planeB.getPointInit());
          int id_B = planeB.getPlaneID();
          if (matched_planes->find(id_B) != matched_planes->end()) {
            if (matched_planes->at(id_B) == planeA.getPlaneID()) {
              continue;
            }
          }
          const float sin_threshold = std::sin(config_.threshold_normal_radians);
          const float sin_threshold_squared = sin_threshold * sin_threshold;
          if (1.0 - normals_cosine*normals_cosine < sin_threshold_squared) {
            if (normals_cosine < 0.0) {
              if (distance_along_normalA < config_.threshold_normal_reversed_distance) {
                // add plane pair
                matched_planes->insert({planeA.getPlaneID(), id_B});
                const PlaneType * planeB = all_planes_.at(id_B).get(); 
                visualizeVector(planeB->getPointInit() - planeA.getPointInit(), planeA.getPointInit());
              }
            } else if (distance_along_normalA < config_.threshold_normal_distance) {
              // add plane pair
              matched_planes->insert({planeA.getPlaneID(), id_B});
              const PlaneType * planeB = all_planes_.at(id_B).get(); 
              visualizeVector(planeB->getPointInit() - planeA.getPointInit(), planeA.getPointInit());
            }
          }
        }
      }
    }
  }
  visual_tools_->trigger();
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
  LOG(ERROR) << "Number of submap neighboors:" << n << " out of "
             << all_submaps.size() << " submaps";
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

void SubmapStitcher::visualizeVector(const Eigen::Vector3f vec,
                                         const Eigen::Vector3f point) const {
  CHECK(visual_tools_);
  static int marker_id = 10000;
  geometry_msgs::Point point_msg;
  geometry_msgs::Point vec_end_msg;
  tf::pointEigenToMsg(point.cast<double>(), point_msg);
  tf::pointEigenToMsg((point + vec).cast<double>(), vec_end_msg);
  visual_tools_->publishArrow(
      point_msg, vec_end_msg, rviz_visual_tools::BLACK,
      rviz_visual_tools::MEDIUM, marker_id++);
}

}  // namespace voxgraph
