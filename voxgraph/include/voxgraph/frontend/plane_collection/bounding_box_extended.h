#ifndef VOXGRAPH_FRONTEND_SUBMAP_COLLECTION_BOUNDING_BOX_EXTENDED_H_
#define VOXGRAPH_FRONTEND_SUBMAP_COLLECTION_BOUNDING_BOX_EXTENDED_H_

#include <algorithm>
#include <utility>

#include <eigen_conversions/eigen_msg.h>
#include <panoptic_mapping_msgs/BoundingBox.h>
#include <visualization_msgs/Marker.h>
#include <voxblox/core/common.h>

#include "voxgraph/common.h"
#include "voxgraph/frontend/submap_collection/bounding_box.h"

namespace voxgraph {

using Point = voxblox::Point;

typedef panoptic_mapping_msgs::BoundingBox BoundingBoxMsg;
class BoundingBoxExtended : public voxgraph::BoundingBox {
 public:
  typedef const BoundingBoxExtended * ConstPtr;
  BoundingBoxExtended() = default;
  ~BoundingBoxExtended() = default;
  void updateBoundingBoxLimits(const Point& p);
  /**
   * @brief Computes the intersection over Union between this box
   * and another one
   * @param other_bounding_box the other bounding box
   * @return float IoU metric
   */
  float IoU(voxgraph::BoundingBox* other_bounding_box) const;

  BoundingBoxMsg toBoundingBoxMsg() const;

  static BoundingBoxExtended fromMsg(const BoundingBoxMsg& msg);

  visualization_msgs::Marker getVisualizationMsg() const;

 protected:
  float computeVolume(const Point& max, const Point& min) const {
    return (max - min).prod();
  }
};

}  // namespace voxgraph

#endif  // VOXGRAPH_FRONTEND_SUBMAP_COLLECTION_BOUNDING_BOX_EXTENDED_H_
