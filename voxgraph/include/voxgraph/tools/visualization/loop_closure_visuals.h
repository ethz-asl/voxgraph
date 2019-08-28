#ifndef VOXGRAPH_TOOLS_VISUALIZATION_LOOP_CLOSURE_VISUALS_H_
#define VOXGRAPH_TOOLS_VISUALIZATION_LOOP_CLOSURE_VISUALS_H_

#include <string>

#include "voxgraph/backend/pose_graph.h"

namespace voxgraph {
class LoopClosureVisuals {
 public:
  LoopClosureVisuals() = default;

  void publishLink(const Transformation& T_W_1, const Transformation& T_W_2,
                   const std::string& frame_id,
                   const ros::Publisher& publisher) const;

  void publishAxes(const Transformation& T_W_1, const Transformation& T_W_2,
                   const std::string& frame_id,
                   const ros::Publisher& publisher) const;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_TOOLS_VISUALIZATION_LOOP_CLOSURE_VISUALS_H_
