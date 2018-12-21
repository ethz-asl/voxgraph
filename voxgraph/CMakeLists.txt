set(ROS_BUILD_TYPE Debug)

cmake_minimum_required(VERSION 2.8.3)
project(voxgraph)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})

############
# BINARIES #
############
cs_add_executable(registration_test_bench
    src/registration_test_bench.cpp
    src/submap_registration/submap_registerer.cpp
    src/submap_registration/registration_cost_function.cpp
    src/visualization.cpp
)
target_link_libraries(registration_test_bench ${catkin_LIBRARIES})
