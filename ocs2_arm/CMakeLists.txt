cmake_minimum_required(VERSION 3.1.3)
project(ocs2_arm)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    urdf
    controller_manager
    joint_state_controller
    robot_state_publisher
)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
