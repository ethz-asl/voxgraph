#include "voxgraph/backend/node/node.h"

namespace voxgraph {
void Node::addToProblem(ceres::Problem* problem,
                        ceres::LocalParameterization* local_parameterization) {
  CHECK_NOTNULL(problem);
  CHECK_NOTNULL(local_parameterization);

  // Add the node to the solver and set it to be held constant if appropriate
  problem->AddParameterBlock(optimized_pose_.optimizationVectorData(),
                             optimized_pose_.optimizationVectorSize());
  if (config_.set_constant) {
    problem->SetParameterBlockConstant(
        optimized_pose_.optimizationVectorData());
  }

  // Set the local parameterization s.t. yaw stays normalized
  problem->SetParameterization(getPosePtr()->optimizationVectorData(),
                               local_parameterization);
}
}  // namespace voxgraph
