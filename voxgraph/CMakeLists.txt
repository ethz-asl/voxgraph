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
    src/frontend/submap_collection/submap_timeline.cpp
    src/frontend/voxgraph_mapper.cpp
    src/frontend/submap_collection/bounding_box.cpp
    src/frontend/submap_collection/voxgraph_submap.cpp
    src/backend/pose_graph.cpp
    src/backend/node_collection.cpp
    src/backend/node/node.cpp
    src/backend/node/submap_node.cpp
    src/backend/node/reference_frame_node.cpp
    src/backend/constraint/constraint.cpp
    src/backend/constraint/odometry_constraint.cpp
    src/backend/constraint/loop_closure_constraint.cpp
    src/backend/constraint/absolute_pose_constraint.cpp
    src/backend/constraint/registration_constraint.cpp
    src/backend/constraint/cost_functions/submap_registration/submap_registerer.cpp
    src/backend/constraint/cost_functions/submap_registration/registration_cost_function_xyz.cpp
    src/backend/constraint/cost_functions/submap_registration/registration_cost_function_xyz_yaw.cpp
    src/tools/tf_helper.cpp
    src/tools/visualization/cost_function_visuals.cpp
    src/tools/visualization/pose_graph_visuals.cpp
    src/tools/visualization/submap_visuals.cpp
    src/tools/odometry_simulator/odometry_simulator.cpp
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
