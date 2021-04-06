#ifndef VOXGRAPH_COMMON_H_
#define VOXGRAPH_COMMON_H_

#include <limits>
#include <utility>

#include <cblox/core/submap_collection.h>

namespace voxgraph {
using SubmapID = cblox::SubmapID;
constexpr SubmapID kInvalidSubmapId = std::numeric_limits<SubmapID>::max();

using Transformation = voxblox::Transformation;
using SubmapIdPair = std::pair<SubmapID, SubmapID>;
using TransformationD = kindr::minimal::QuatTransformationTemplate<double>;
using BiasVectorType = TransformationD::Vector3;
}  // namespace voxgraph

#endif  // VOXGRAPH_COMMON_H_
