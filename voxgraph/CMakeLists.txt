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
    src/frontend/voxgraph_mapper.cpp
    src/frontend/pose_graph_interface.cpp
    src/frontend/measurement_processors/pointcloud_processor.cpp
    src/frontend/submap_collection/bounding_box.cpp
    src/frontend/submap_collection/submap_timeline.cpp
    src/frontend/submap_collection/voxgraph_submap.cpp
    src/frontend/submap_collection/voxgraph_submap_collection.cpp
    src/backend/pose_graph.cpp
    src/backend/node/node.cpp
    src/backend/node/node_collection.cpp
    src/backend/constraint/constraint.cpp
    src/backend/constraint/constraint_collection.cpp
    src/backend/constraint/relative_pose_constraint.cpp
    src/backend/constraint/absolute_pose_constraint.cpp
    src/backend/constraint/registration_constraint.cpp
    src/backend/constraint/cost_functions/submap_registration/registration_cost.cpp
    src/backend/constraint/cost_functions/submap_registration/implicit_implicit_registration_cost.cpp
    src/backend/constraint/cost_functions/submap_registration/explicit_implicit_registration_cost.cpp
    src/tools/rosbag_helper.cpp
    src/tools/tf_helper.cpp
    src/tools/submap_registration_helper.cpp
    src/tools/evaluation/map_evaluation.cpp
    src/tools/odometry_simulator/odometry_simulator.cpp
    src/tools/visualization/cost_function_visuals.cpp
    src/tools/visualization/pose_graph_visuals.cpp
    src/tools/visualization/submap_visuals.cpp
)

############
# BINARIES #
############
cs_add_executable(voxgraph_mapping_node
    src/voxgraph_mapping_node.cpp)
target_link_libraries(voxgraph_mapping_node ${PROJECT_NAME})

cs_add_executable(odometry_simulator
    src/odometry_simulator_node.cpp)
target_link_libraries(odometry_simulator ${PROJECT_NAME})

cs_add_executable(registration_test_bench
    src/tools/test_benches/registration_test_bench.cpp)
target_link_libraries(registration_test_bench ${PROJECT_NAME})

cs_add_executable(pose_graph_test_bench
    src/tools/test_benches/pose_graph_test_bench.cpp)
target_link_libraries(pose_graph_test_bench ${PROJECT_NAME})

cs_add_executable(mapper_test_bench
    src/tools/test_benches/mapper_test_bench.cpp)
target_link_libraries(mapper_test_bench ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_export()
