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
    src/submap_registration/registration_cost_function_xyz.cpp
    src/submap_registration/registration_cost_function_xyz_yaw.cpp
    src/visualization/submap_visuals.cpp
    src/visualization/cost_function_visuals.cpp
    src/utils/tf_helper.cpp)

cs_add_executable(pose_graph_test_bench
    src/pose_graph_test_bench.cpp
    src/mapper/voxgraph_submap.cpp
    src/pose_graph/pose_graph.cpp
    src/pose_graph/node/submap_node.cpp
    src/pose_graph/constraint/registration_constraint.cpp
    src/submap_registration/submap_registerer.cpp
    src/submap_registration/registration_cost_function_xyz.cpp
    src/submap_registration/registration_cost_function_xyz_yaw.cpp
    src/visualization/submap_visuals.cpp
    src/visualization/cost_function_visuals.cpp
    src/visualization/pose_graph_visuals.cpp
    src/utils/tf_helper.cpp)

#cs_add_executable(voxgraph_mapping_node
#    src/voxgraph_mapping_node.cpp
#    src/mapper/voxgraph_mapper.cpp)

cs_add_executable(voxgraph_odometry_simulator
    src/odometry_simulator_node.cpp
    src/odometry_simulator/odometry_simulator.cpp)