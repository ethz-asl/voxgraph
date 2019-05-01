//
// Created by victor on 28.04.19.
//

#ifndef VOXGRAPH_FRONTEND_SUBMAP_COLLECTION_REGISTRATION_POINT_H_
#define VOXGRAPH_FRONTEND_SUBMAP_COLLECTION_REGISTRATION_POINT_H_

namespace voxgraph {
// Small struct that can be used to hold voxels or points on the isosurface
struct RegistrationPoint {
  voxblox::Point position;
  float distance;
  float weight;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_FRONTEND_SUBMAP_COLLECTION_REGISTRATION_POINT_H_
