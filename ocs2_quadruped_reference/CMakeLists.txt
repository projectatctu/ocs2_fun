cmake_minimum_required(VERSION 3.0.2)
project(ocs2_quadruped_reference)

find_package(catkin REQUIRED COMPONENTS
  roslib
  sensor_msgs
  ocs2_ros_interfaces
  ocs2_robotic_tools
  ocs2_switched_model_interface
  ocs2_anymal_commands
  grid_map_filters_rsl
  grid_map_core
  grid_map_ros
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

catkin_package(
  INCLUDE_DIRS 
    include
    ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ocs2_ros_interfaces
    ocs2_robotic_tools
    ocs2_switched_model_interface
    ocs2_anymal_commands
  DEPENDS
)


include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/ReferenceGenerator.cpp
)
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

add_executable(joystick_reference_generator_node
  src/JoystickReferenceGeneratorNode.cpp
)
add_dependencies(joystick_reference_generator_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(joystick_reference_generator_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_compile_options(joystick_reference_generator_node PRIVATE ${OCS2_CXX_FLAGS})

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
install(TARGETS joystick_reference_generator_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
