#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('simple_move')

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("arm")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

pose_target = geometry_msgs.msg.Pose()
pose_target.position.x = 0.5
pose_target.position.y = 0.5
pose_target.position.z = 0.5
pose_target.orientation.w = 1.0

group.set_pose_target(pose_target)
plan = group.plan()

rospy.sleep(5)
moveit_commander.roscpp_shutdown()