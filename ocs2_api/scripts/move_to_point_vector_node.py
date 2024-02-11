#!/usr/bin/env python3

import rospy
from ocs2_api.move_to_point_vector import MoveToPoint

def main():
    print("here")
    rospy.init_node('Move_to_point_vector_started')
    MoveToPoint().move_to_point()
    rospy.spin()

if __name__ == '__main__':
    main()