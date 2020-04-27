#ifndef VOXGRAPH_COMMON_H_
#define VOXGRAPH_COMMON_H_

#include <utility>

#include <cblox/core/submap_collection.h>

#include "voxgraph/frontend/submap_collection/voxgraph_submap.h"

namespace voxgraph {
using Transformation = voxblox::Transformation;
using SubmapID = cblox::SubmapID;
using SubmapIdPair = std::pair<SubmapID, SubmapID>;
using TransformationD = kindr::minimal::QuatTransformationTemplate<double>;
using BiasVectorType = TransformationD::Vector3;
}  // namespace voxgraph

#endif  // VOXGRAPH_COMMON_H_
