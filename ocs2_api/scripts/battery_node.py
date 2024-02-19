#!/usr/bin/env python3

import rospy
from ocs2_api.battery import Battery

def main():
    rospy.init_node('Battery_status')
    Battery().battery_usage()
    rospy.spin()

if __name__ == '__main__':
    main()