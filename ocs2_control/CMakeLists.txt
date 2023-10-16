cmake_minimum_required(VERSION 3.0.2)
project(ocs2_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    qpoases_catkin
    ocs2_mpc
    ocs2_core
    ocs2_gazebo
    ocs2_anymal_mpc
    ocs2_ros_interfaces
    ocs2_pinocchio_interface
    ocs2_quadruped_interface
    ocs2_switched_model_interface
)
find_package(pinocchio REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${pinocchio_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS 
  LIBRARIES
  CATKIN_DEPENDS
  DEPENDS
)


add_library(${PROJECT_NAME}
    src/wbc/Task.cpp
    src/wbc/SqpSolver.cpp
    src/wbc/SqpWbc.cpp
    src/wbc/WbcBase.cpp
    src/simple/StandController.cpp
    src/utils/LoadConfig.cpp
)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${pinocchio_LIBRARIES}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

add_executable(wbc_controller_node src/AnymalControllerNode.cpp)
target_link_libraries(wbc_controller_node ${catkin_LIBRARIES} ${PROJECT_NAME})
target_compile_options(wbc_controller_node PRIVATE ${OCS2_CXX_FLAGS})
