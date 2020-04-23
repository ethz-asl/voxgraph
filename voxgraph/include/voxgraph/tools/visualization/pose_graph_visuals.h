#ifndef VOXGRAPH_TOOLS_VISUALIZATION_POSE_GRAPH_VISUALS_H_
#define VOXGRAPH_TOOLS_VISUALIZATION_POSE_GRAPH_VISUALS_H_

#include <string>

#include "voxgraph/backend/pose_graph.h"

namespace voxgraph {
class PoseGraphVisuals {
 public:
  PoseGraphVisuals() = default;

  void publishPoseGraph(const PoseGraph& pose_graph,
                        const std::string& frame_id,
                        const std::string& name_space,
                        const ros::Publisher& publisher) const;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_TOOLS_VISUALIZATION_POSE_GRAPH_VISUALS_H_
