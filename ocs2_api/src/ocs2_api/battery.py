#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

class Battery:
    def __init__(self):
        self.battery = 100

    def battery_usage(self):
        battery_pub = rospy.Publisher('battery', Int32, queue_size=10)
        rate = rospy.Rate(0.05)
        while not rospy.is_shutdown():
            if self.battery > 0:
                self.battery -= 1
            battery_pub.publish(self.battery)
            rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('Batery_usage')
        Battery().battery_usage()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
