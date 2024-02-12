#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

def main():
    height_pub = rospy.Publisher('height', Float32, queue_size=10)
    rospy.init_node('Height')
    height = Float32()
    height.data = user_input()
    height.data = max(30.0, min(height.data, 60.0))
    height.data /= 100
    height_pub.publish(height)

def user_input():
    correct_input = False
    while not correct_input:
        try:
            height = float(input("Enter height from 30 to 60 cm:"))
            correct_input = True
        except ValueError:
            print("Invalid input. Please enter number between 30 and 60.")
    return height

if __name__ == '__main__':
    main()