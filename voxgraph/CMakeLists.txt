cmake_minimum_required(VERSION 2.8.3)
project(voxgraph)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#############
# LIBRARIES #
#############
# TODO(victorr): Use library with all deps, then link against it

############
# BINARIES #
############
cs_add_executable(registration_test_bench
    src/registration_test_bench.cpp
    src/submap_registration/submap_registerer.cpp
    src/submap_registration/registration_cost_function.cpp
    src/visualization/submap_visuals.cpp
    src/visualization/cost_function_visuals.cpp
    src/visualization/tf_helper.cpp)

cs_add_executable(pose_graph_test_bench
    src/pose_graph_test_bench.cpp
    src/pose_graph/pose_graph.cpp
    src/pose_graph/submap_node.cpp
    src/pose_graph/registration_constraint.cpp
    src/submap_registration/submap_registerer.cpp
    src/submap_registration/registration_cost_function.cpp
    src/visualization/submap_visuals.cpp
    src/visualization/cost_function_visuals.cpp
    src/visualization/tf_helper.cpp)
