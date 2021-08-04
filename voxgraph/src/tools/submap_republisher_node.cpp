#include <gflags/gflags.h>

#include "voxgraph/tools/submap_republisher.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "voxgraph_submap_republisher");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  voxgraph::SubmapRepublisher node(nh, nh_private);

  ros::spin();
  return 0;
}
