//
// Created by victor on 04.12.18.
//

#ifndef VOXGRAPH_SUBMAP_REGISTRATION_H
#define VOXGRAPH_SUBMAP_REGISTRATION_H

#include <ceres/ceres.h>
#include <cblox/core/common.h>
#include <cblox/core/tsdf_submap_collection.h>

namespace voxgraph {
class SubmapRegisterer {
public:
  struct Options {
    ceres::Solver::Options solver;
    struct CostFunction {
      enum Type {kAnalytic=0, kNumeric} cost_function_type;
      double min_voxel_weight = 0;
      double max_voxel_distance = 1;
      double no_correspondence_cost = 0; // Cost to assign for voxels that can't be interpolated
      bool visualize_residuals = false;
    } cost;
  };

  SubmapRegisterer(cblox::TsdfSubmapCollection::ConstPtr tsdf_submap_collection_ptr, Options& options);

  ~SubmapRegisterer() = default;

  bool findRegistration(const cblox::SubmapID &reference_submap_id,
                        const cblox::SubmapID &reading_submap_id,
                        double *ref_t_ref_reading,
                        ceres::Solver::Summary &summary);

private:
  cblox::TsdfSubmapCollection::ConstPtr tsdf_submap_collection_ptr_;
  Options options_;
};
} // namespace voxgraph

#endif //VOXGRAPH_SUBMAP_REGISTRATION_H
