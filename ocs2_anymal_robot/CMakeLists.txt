cmake_minimum_required(VERSION 3.0.2)
project(ocs2_anymal_robot)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roslib
  ocs2_anymal_models
  ocs2_anymal_mpc
  ocs2_quadruped_interface
)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
    ocs2_anymal_mpc
    ocs2_anymal_models
    ocs2_quadruped_interface
  DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(anymal_mpc_node
  src/AnymalMpcNode.cpp
)
target_link_libraries(anymal_mpc_node
  ${catkin_LIBRARIES}
)
target_compile_options(anymal_mpc_node PRIVATE ${OCS2_CXX_FLAGS})
