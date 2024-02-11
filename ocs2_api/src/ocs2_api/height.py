#!/usr/bin/env python
import sys
sys.path.append("/opt/ros/noetic/lib/python3/dist-packages")

import rospy
from std_msgs.msg import Float32

def main():
    height_pub = rospy.Publisher('height', Float32, queue_size=10)
    rospy.init_node('Height')
    print("Input height 20 to 60 cm:")
    height = Float32()
    height.data = float(input())
    if height.data > 60:
        height.data = 60
    if height.data < 20:
        height.data = 20
    height_pub.publish(height)

if __name__ == '__main__':
    main()