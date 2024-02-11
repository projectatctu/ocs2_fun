#!/usr/bin/env python
import sys
sys.path.append("/opt/ros/noetic/lib/python3/dist-packages")

import rospy
from std_msgs.msg import Float32MultiArray

def main():
    tilt_pub = rospy.Publisher('/tilt', Float32MultiArray, queue_size=10)
    rospy.init_node('Tilt')
    print("Input tilt -20 to 20 degrees:")
    tilt = input()
    print("Input tilt direction 0 to 360 degrees")
    tilt_dir = input()
    tilt = float(tilt)
    tilt_dir = float(tilt_dir)
    if tilt < -20:
        tilt = -20
    if tilt > 20:
        tilt = 20
    if tilt_dir < 0:
        tilt_dir = 0
    if tilt_dir > 360:
        tilt_dir = 360
    arr = Float32MultiArray()
    arr.data = [tilt,tilt_dir]
    tilt_pub.publish(arr)

if __name__ == '__main__':
    main()

