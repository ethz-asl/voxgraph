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
    src/voxgraph_submap.cpp
    src/pose_graph/pose_graph.cpp
    src/pose_graph/constraint/registration_constraint.cpp
    src/pose_graph/node/submap_node.cpp
    src/submap_registration/submap_registerer.cpp
    src/submap_registration/registration_cost_function_xyz.cpp
    src/submap_registration/registration_cost_function_xyz_yaw.cpp
    src/visualization/cost_function_visuals.cpp
    src/visualization/pose_graph_visuals.cpp
    src/visualization/submap_visuals.cpp
    src/visualization/tf_helper.cpp
)

############
# BINARIES #
############
cs_add_executable(registration_test_bench
    src/registration_test_bench.cpp
)
target_link_libraries(registration_test_bench ${PROJECT_NAME})

cs_add_executable(pose_graph_test_bench
    src/pose_graph_test_bench.cpp
)
target_link_libraries(pose_graph_test_bench ${PROJECT_NAME})