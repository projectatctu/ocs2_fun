cmake_minimum_required(VERSION 3.0.2)
project(ocs2_gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
    message_generation
    controller_interface
    hardware_interface
    pluginlib
    roscpp
    urdf
    realtime_tools
    std_msgs
    ocs2_anymal_commands
)

find_package(gazebo REQUIRED)

add_message_files(
  FILES
  JointCommand.msg
  JointCommandArray.msg
  RobotState.msg
)
generate_messages()

catkin_package(
  INCLUDE_DIRS 
    include
    ${GAZEBO_INCLUDE_DIRS}
  LIBRARIES
  CATKIN_DEPENDS
    ocs2_robotic_tools
    ocs2_switched_model_interface
    ocs2_anymal_commands
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(JointController src/JointController.cpp)
target_link_libraries(JointController ${catkin_LIBRARIES}) 
target_compile_options(JointController PRIVATE ${OCS2_CXX_FLAGS})

# Anymal state estimator plugin
add_library(StateEstimator src/StateEstimator.cpp)
target_link_libraries(StateEstimator ${gazebo_LIBRARIES} ${catkin_LIBRARIES})
target_compile_options(StateEstimator PRIVATE ${OCS2_CXX_FLAGS})

# Anymal contact sensor
add_library(ContactSensor SHARED src/ContactSensor.cpp)
target_link_libraries(ContactSensor ${gazebo_LIBRARIES})
target_compile_options(ContactSensor PRIVATE ${OCS2_CXX_FLAGS})

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(TARGETS JointController StateEstimator ContactSensor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )