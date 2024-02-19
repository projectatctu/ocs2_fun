#!/usr/bin/env python3

import rospy
from ocs2_api.estop import Estop_status

def main():
    rospy.init_node('Estop_status')
    Estop_status().main()
    rospy.spin()

if __name__ == '__main__':
    main()