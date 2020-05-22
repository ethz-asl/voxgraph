# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;voxblox;voxblox_ros;cblox;ceres_catkin;eigen_catkin;voxblox_msgs;voxgraph_msgs;maplab_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lvoxgraph".split(';') if "-lvoxgraph" != "" else []
PROJECT_NAME = "voxgraph"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "1.0.0"
