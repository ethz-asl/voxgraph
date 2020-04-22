#ifndef VOXGRAPH_TOOLS_ODOMETRY_SIMULATOR_NORMAL_DISTRIBUTION_H_
#define VOXGRAPH_TOOLS_ODOMETRY_SIMULATOR_NORMAL_DISTRIBUTION_H_

#include <random>

#include <glog/logging.h>

namespace voxgraph {
class NormalDistribution {
 public:
  explicit NormalDistribution(const double& mean = 0, const double& stddev = 1)
      : mean_(mean), stddev_(stddev) {}

  double& mean() { return mean_; }
  double& stddev() { return stddev_; }

  // Return a sample from the normal distribution N(mean_, stddev_)
  double operator()() {
    CHECK_GE(stddev_, 0) << "The standard deviation must be non-negative.";
    // Random engine
    // TODO(victorr): Add proper random seed handling (and option to provide it)
    // NOTE: The noise generator is static to ensure that all instances draw
    //       subsequent (different) numbers from the same pseudo random
    //       sequence. If the generator is instance specific, there's a risk
    //       that multiple instances use generators with the same seed and
    //       output the same sequence.
    static std::mt19937 noise_generator_;

    // Draw a sample from the standard normal N(0,1) and
    // scale it using the change of variables formula
    return normal_distribution_(noise_generator_) * stddev_ + mean_;
  }

 private:
  double mean_, stddev_;

  // Standard normal distribution
  std::normal_distribution<double> normal_distribution_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_TOOLS_ODOMETRY_SIMULATOR_NORMAL_DISTRIBUTION_H_
