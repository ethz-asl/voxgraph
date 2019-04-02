cmake_minimum_required(VERSION 2.8.3)
project(voxgraph)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#############
# LIBRARIES #
#############
# NOTE: The library is static to avoid segfaults
#       following voxblox multi threading
cs_add_library(${PROJECT_NAME} STATIC
    src/mapper/submap_collection/submap_timeline.cpp
    src/mapper/voxgraph_mapper.cpp
    src/mapper/submap_collection/voxgraph_submap.cpp
    src/tools/odometry_simulator/odometry_simulator.cpp
    src/pose_graph/pose_graph.cpp
    src/pose_graph/constraint/registration_constraint.cpp
    src/pose_graph/node/submap_node.cpp
    src/pose_graph/constraint/submap_registration/submap_registerer.cpp
    src/pose_graph/constraint/submap_registration/registration_cost_function_xyz.cpp
    src/pose_graph/constraint/submap_registration/registration_cost_function_xyz_yaw.cpp
    src/tools/tf_helper.cpp
    src/tools/visualization/cost_function_visuals.cpp
    src/tools/visualization/pose_graph_visuals.cpp
    src/tools/visualization/submap_visuals.cpp
)

############
# BINARIES #
############
cs_add_executable(registration_test_bench
    src/registration_test_bench.cpp)
target_link_libraries(registration_test_bench ${PROJECT_NAME})

cs_add_executable(pose_graph_test_bench
    src/pose_graph_test_bench.cpp)
target_link_libraries(pose_graph_test_bench ${PROJECT_NAME})

cs_add_executable(odometry_simulator
    src/odometry_simulator_node.cpp)
target_link_libraries(odometry_simulator ${PROJECT_NAME})

cs_add_executable(voxgraph_mapping_node
    src/voxgraph_mapping_node.cpp)
target_link_libraries(voxgraph_mapping_node ${PROJECT_NAME})
