#ifndef VOXGRAPH_TOOLS_THREADING_HELPER_H_
#define VOXGRAPH_TOOLS_THREADING_HELPER_H_

#include <functional>
#include <thread>
#include <utility>

namespace voxgraph {
class ThreadingHelper {
 public:
  template <typename Function, typename... Args>
  static void launchBackgroundThread(Function&& function, Args&&... args) {
    // Bind the function to avoid having to distinguish between special cases
    // when calling it later (member functions,...)
    // NOTE: Unfortunately std::invoke is not yet available in C++11
    auto bound_function = std::bind(function, args...);

    // Run the thread in the background by giving it a high nice value
    std::thread background_thread([bound_function] {
      if (nice(20) == -1) {
        ROS_WARN_STREAM(
            "Could not deprioritize thread to "
            "run in the background: "
            << strerror(errno));
      }
      bound_function();
    });
    background_thread.detach();
  }
};
}  // namespace voxgraph

#endif  // VOXGRAPH_TOOLS_THREADING_HELPER_H_
