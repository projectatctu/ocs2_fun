#!/usr/bin/env python
import rospy
from ocs2_api.msg import TiltCommand
import numpy as np

def main():
    tilt_pub = rospy.Publisher('/tilt', TiltCommand, queue_size=10)
    #tilt_dir_pub = rospy.Publisher('/tilt_dir', TiltCommand, queue_size=10)
    rospy.init_node('Tilt')
    tilt = user_input_tilt()
    tilt_dir = user_input_tilt_direction()
    tilt = max(-20.0, min(tilt, 20.0))
    tilt = tilt*np.pi/180
    tilt_dir = max(0.0, min(tilt_dir, 360.0))
    tilt_dir = tilt_dir*np.pi/180
    """desired_tilt = Float32()
    desired_tilt_direction = Float32()"""
    new_tilt_command = TiltCommand()
    new_tilt_command.tilt = tilt
    new_tilt_command.tilt_direction = tilt_dir
    tilt_pub.publish(new_tilt_command)
    #tilt_dir_pub.publish(desired_tilt_direction)

def user_input_tilt():
    correct_input = False
    while not correct_input:
        try:
            input_float = float(input("Enter tilt from -20 to 20 degrees: "))
            correct_input = True
        except ValueError:
            print("Invalid input. Enter number between -20 and 20")
    return input_float

def user_input_tilt_direction():
    correct_input = False
    while not correct_input:
        try:
            input_float = float(input("Enter direction of tilt from 0 to 360 degrees: "))
            correct_input = True
        except ValueError:
            print("Invalid input. Enter number between 0 to 360 degrees")
    return input_float





if __name__ == '__main__':
    main()

