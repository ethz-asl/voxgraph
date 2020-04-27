#ifndef VOXGRAPH_TOOLS_VISUALIZATION_LOOP_CLOSURE_VISUALS_H_
#define VOXGRAPH_TOOLS_VISUALIZATION_LOOP_CLOSURE_VISUALS_H_

#include <string>

#include <visualization_msgs/Marker.h>

#include "voxgraph/backend/pose_graph.h"

namespace voxgraph {
class LoopClosureVisuals {
 public:
  struct Color {
    Color() : r(0), g(0), b(0), a(0) {}
    Color(float _r, float _g, float _b) : Color(_r, _g, _b, 1.0) {}
    Color(float _r, float _g, float _b, float _a)
        : r(_r), g(_g), b(_b), a(_a) {}

    float r;
    float g;
    float b;
    float a;
  };

  LoopClosureVisuals() = default;

  void publishLoopClosure(const Transformation& T_W_1,
                          const Transformation& T_W_2,
                          const Transformation& T_1_2_est,
                          const std::string& frame_id,
                          const ros::Publisher& publisher) const;

  visualization_msgs::Marker getLinkMarker(const Transformation& T_W_1,
                                           const Transformation& T_W_2,
                                           const std::string& frame_id,
                                           const Color& color,
                                           const std::string& ns) const;

  void publishAxes(const Transformation& T_W_1, const Transformation& T_W_2,
                   const Transformation& T_1_2_est, const std::string& frame_id,
                   const ros::Publisher& publisher) const;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_TOOLS_VISUALIZATION_LOOP_CLOSURE_VISUALS_H_
