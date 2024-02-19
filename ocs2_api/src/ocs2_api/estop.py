#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool


class Estop_status:
    def __init__(self):
        self.estop = False
    def main(self):
        estop_pub = rospy.Publisher('estop', Bool, queue_size=10)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            estop = Bool()
            estop.data = False
            estop_pub.publish(estop)
            rate.sleep()
