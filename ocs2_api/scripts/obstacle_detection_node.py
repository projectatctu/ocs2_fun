#!/usr/bin/env python3

import rospy
from ocs2_api.obstacle_detection import PCLFromGrid

def main():
    rospy.init_node('Obstacle_detection')
    PCLFromGrid().get_pointcloud_map()
    rospy.spin()

if __name__ == '__main__':
    main()