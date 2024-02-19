#!/usr/bin/env python
import sys
sys.path.append("/opt/ros/noetic/lib/python3/dist-packages")

import keyboard
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


def keyboard_detection():
    twist_pub = rospy.Publisher('twist_cmd', Twist, queue_size=10)
    string_pub = rospy.Publisher('keyboard_det', String, queue_size=10)
    rospy.init_node('keyboard_detection')
    rate = rospy.Rate(10)
    last_move = [0,0,0]  #x,y,phi(z)
    while not rospy.is_shutdown():
        move = Twist()
        string = String()


        if keyboard.is_pressed('h'):
            string.data = 'Tomáš'
            string_pub.publish(string)
        if keyboard.is_pressed('j'):
            string.data = 'Kuba'
            string_pub.publish(string)
        if keyboard.is_pressed('k'):
            string.data = 'Spot'
            string_pub.publish(string)

        if (keyboard.is_pressed("up") and keyboard.is_pressed("down") or
            keyboard.is_pressed("right") and keyboard.is_pressed("left") or
            keyboard.is_pressed("a") and keyboard.is_pressed("d")):
            continue

        if keyboard.is_pressed("up"):
            x_velocity = acceleratrion(last_move, axis=0, direction=1)
            last_move[0] = x_velocity
            #move.linear.x = 0.5
        if keyboard.is_pressed("down"):
            x_velocity = acceleratrion(last_move, axis=0, direction=-1)
            last_move[0] = x_velocity
            #move.linear.x = -0.5

        if keyboard.is_pressed("left"):
            y_velocity = acceleratrion(last_move, axis=1, direction=1)
            last_move[1] = y_velocity

            #move.linear.y = 0.5
        if keyboard.is_pressed("right"):
            y_velocity = acceleratrion(last_move, axis=1, direction=-1)
            last_move[1] = y_velocity
            #move.linear.y = -0.5

        if keyboard.is_pressed("a"):
            phi_velocity = acceleratrion(last_move, axis=2, direction=1)
            last_move[2] = phi_velocity
            #move.angular.z = 0.3
        if keyboard.is_pressed("d"):
            phi_velocity = acceleratrion(last_move, axis=2, direction=-1)
            last_move[2] = phi_velocity
            # move.angular.z = - 0.3

        if not keyboard.is_pressed("up") and not keyboard.is_pressed("down"):
            stopping(last_move, axis=0)

        if not keyboard.is_pressed("right") and not keyboard.is_pressed("left"):
            stopping(last_move, axis=1)

        if not keyboard.is_pressed("d") and not keyboard.is_pressed("a"):
            stopping(last_move, axis=2)
        move.linear.x, move.linear.y, move.angular.z = last_move
        twist_pub.publish(move)
        #print(data)
        rate.sleep()

def acceleratrion(last_move, axis, direction):
    last_velocity = last_move[axis]
    max_velocity_straight = 0.3
    min_velocity_straight = -0.3
    max_velocity_side = 0.3
    min_velocity_side = -0.3
    max_angular = 0.5
    min_angular = -0.5
    new_velocity = 0.0

    if last_velocity >= 0 and direction < 0:
        new_velocity = last_velocity - 0.08

    elif last_velocity < 0 and direction > 0:
        new_velocity = last_velocity + 0.08

    elif last_velocity >= 0 and direction > 0:
        new_velocity = (max_velocity_straight - last_velocity)/10 + last_velocity
        if axis == 2:
            new_velocity = (max_angular - last_velocity)/15 + last_velocity
        if axis == 1:
            new_velocity = (max_velocity_side - last_velocity)/10 + last_velocity

    else:
        new_velocity = (min_velocity_straight - last_velocity)/10 + last_velocity
        if axis == 2:
            new_velocity = (min_angular - last_velocity)/15 + last_velocity
        if axis == 1:
            new_velocity = (min_velocity_side - last_velocity)/10 + last_velocity
    return new_velocity

def stopping(last_move, axis):
    if last_move[axis] > 0:
        last_move[axis] -= 0.05
        if last_move[axis] < 0:
            last_move[axis] = 0
    if last_move[axis] < 0:
        last_move[axis] += 0.05
        if last_move[axis] > 0:
            last_move[axis] = 0

keyboard_detection()
