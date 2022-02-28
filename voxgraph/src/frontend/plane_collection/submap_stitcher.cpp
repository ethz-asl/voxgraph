#include "voxgraph/frontend/plane_collection/submap_stitcher.h"

#include <map>
#include <utility>
#include <vector>

#include "voxgraph/frontend/plane_collection/plane_type.h"
#include "voxgraph/frontend/submap_collection/voxgraph_submap.h"

namespace voxgraph {

SubmapStitcher::SubmapStitcher():nh_("") {
  submap_id_to_class_to_planes_ =
      std::make_shared<std::map<int, classToPlanesType>>();
  vec_pub_ = nh_.advertise<visualization_msgs::Marker>(
      "/voxgraph_mapper/cost_function_vecs", 100);
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
        float dist2_min = +INFINITY;
        int closest_plane_id = -1;
        for (const PlaneType& planeB : planeSeriesB.at(class_id)) {
          float dist2_current = planeB.distSquared(planeA);
          if (dist2_current < dist2_min) {
            dist2_min = dist2_current;
            closest_plane_id = planeB.getPlaneID();
          }
        }
        if (dist2_min < config_.threshold_dist) {
          matched_planes->insert({planeA.getPlaneID(), closest_plane_id});
          const PlaneType * planeB = all_planes_.at(closest_plane_id).get(); 
          visualizeVector(planeB->getPointInit() - planeA.getPointInit(), planeA.getPointInit());
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
  static int marker_id = 10000;
  visualization_msgs::Marker msg;
  msg.header.frame_id = "world";
  msg.type = visualization_msgs::Marker::LINE_LIST;
  msg.action = visualization_msgs::Marker::ADD;
  geometry_msgs::Point point_msg;
  geometry_msgs::Point vec_end_msg;
  tf::pointEigenToMsg(point.cast<double>(), point_msg);
  tf::pointEigenToMsg((point + vec * 1.0).cast<double>(), vec_end_msg);
  msg.points.push_back(point_msg);
  msg.points.push_back(vec_end_msg);
  msg.scale.x = 0.10;
  msg.scale.y = 0.10;
  msg.scale.z = 0.10;
  msg.color.r = 0.0;
  msg.color.g = 0.0;
  msg.color.b = 0.0;
  msg.color.a = 1.0;
  msg.id = marker_id++;
  msg.ns = "vector";
  msg.header.stamp = ros::Time::now();
  vec_pub_.publish(msg);
}

}  // namespace voxgraph
