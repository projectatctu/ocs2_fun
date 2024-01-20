#!/usr/bin/env python3

import rospy
from ocs2_api.move_to_point import Move_to_point

def main():
    rospy.init_node('Move_to_point_started')
    Move_to_point().move_to_point()
    rospy.spin()