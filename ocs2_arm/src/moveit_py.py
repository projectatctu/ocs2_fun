#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import numpy as np
import actionlib

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroup(object):
    def __init__(self):
        super(MoveGroup, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group', anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "arm"
        group = moveit_commander.MoveGroupCommander(group_name)
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)
        self._exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory',
                                                                        moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        planning_frame = group.get_planning_frame()
        print("============ Reference frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = group.get_end_effector_link()
        print("============ End effector: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Robot Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        #       arm joints limits
        self.joint_limit_part0 = [-3.141593, 2.617994]
        self.joint_limit_part1 = [-3.665191, 0.0]
        self.joint_limit_part2 = [0.0, 2.96706]
        self.joint_limit_part3 = [-2.879793, 2.879793]
        self.joint_limit_part4 = [-1.570796, 1.570796]
        #       gripper joints limits
        self.gripper_limit_part0 = [-2.96706, 2.96706]
        self.gripper_limit_part1 = [-1.396263, 0.349066]

    def go_to_joint_state(self):
        group = self.group
        joint_goal = group.get_current_joint_values()
        joint_goal[0] = 0.1 #np.random.uniform(self.joint_limit_part0[0], self.joint_limit_part0[1])
        joint_goal[1] = -0.1 #np.random.uniform(self.joint_limit_part1[0], self.joint_limit_part1[1])
        joint_goal[2] = 0.1 #np.random.uniform(self.joint_limit_part2[0], self.joint_limit_part2[1])
        joint_goal[3] = 0.1 #np.random.uniform(self.joint_limit_part3[0], self.joint_limit_part3[1])
        joint_goal[4] = 0.1 #np.random.uniform(self.joint_limit_part4[0], self.joint_limit_part4[1])

        print("GO:")
        group.go(joint_goal, wait=True)
        group.stop()

        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pos(self):
        group = self.group
        group.set_random_target()
        group.go()
        print(group.get_current_state())
        print("----------------------")
        #print(group.get_active_joints())
        print(group.get_current_joint_values())
        #group.clear_pose_targets()


    def main(self):
        print("Starting random movement")
        for i in range(4):
            #self.go_to_joint_state()
            self.go_to_pos()
        print("Done")

if __name__ == '__main__':
    MoveGroup().main()


