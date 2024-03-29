#!/usr/bin/env python
import sys
sys.path.append("/opt/ros/noetic/lib/python3/dist-packages")

import rospy
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
import numpy as np

class MoveToPoint:
    def __init__(self):
        self.new_x = 0
        self.new_y = 0
        self.new_orientation = 0
        self.curr_position_x = 0
        self.curr_position_y = 0
        self.curr_orientation = 0
        #rospy.init_node('tu')
    def new_state_callback(self,data):
        point = data.pose
        self.new_x = point.position.x
        self.new_y = point.position.y
        self.new_orientation = point.orientation.z

    def curr_state_callback(self,data):
        pose = data.pose[-1]
        #print("here")
        self.curr_position_x = pose.position.x
        self.curr_position_y = pose.position.y
        self.curr_orientation = pose.orientation.z

    def listener(self):
        #rospy.init_node('listener')
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.new_state_callback)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.curr_state_callback)
        #rospy.spin()

    def coord_transformation(self,theta,x,y):
        T = np.array([
            [np.cos(theta), np.sin(theta)],
             [-np.sin(theta), np.cos(theta)]
        ])
        vector = T @ np.array([x, y])
        return vector


    def move_to_point(self):
        twist_pub = rospy.Publisher('twist_cmd', Twist, queue_size=10)
        rate = rospy.Rate(10)
        last_move = [0, 0, 0]
        while not rospy.is_shutdown():
            self.listener()
            if self.new_x == 0 and self.new_y == 0:
                self.new_x = self.curr_position_x
                self.new_y = self.curr_position_y
            move = Twist()
            curr_point = np.array([self.curr_position_x, self.curr_position_y])
            end_point = np.array([self.new_x, self.new_y])
            vector = end_point-curr_point
            print("--------------------------------")
            #print(self.new_orientation,"new_orientation")
            #print(self.curr_orientation,"curr_orientation")

            if np.linalg.norm(end_point - curr_point) > 0.15:
                vector = self.coord_transformation(self.curr_orientation*np.pi, vector[0], vector[1])
                distance = np.linalg.norm(vector)
                vector = (vector/np.linalg.norm(vector))
                #last_move[:2] = self.coord_transformation(self.curr_orientation*np.pi, last_move[0], last_move[1])
                print(last_move,"last_move 1")
                last_move = self.set_speed(distance, last_move, vector)
                print(last_move,"last_move 2")
                if vector[0] > 0.9:
                    move.linear.x = 0.3
                else:
                    move.linear.x = vector[0] * 0.3
                move.linear.y = vector[1] * 0.2
                """if vector[0] > 0:
                    last_move[0] = self.acceleratrion(vector, last_move, axis=0, direction=1)
                elif vector[0] < 0:
                    last_move[0] = self.acceleratrion(vector, last_move, axis=0, direction=-1)
                if vector[1] > 0:
                    last_move[1] = self.acceleratrion(vector, last_move, axis=1, direction=1)
                elif vector[1] < 0:
                    last_move[1] = self.acceleratrion(vector, last_move, axis=1, direction=-1)
            else:
                if np.linalg.norm(end_point - curr_point) > 0.05:
                    vector = self.coord_transformation(self.curr_orientation * np.pi, vector[0], vector[1])
                    vector = (vector / np.linalg.norm(vector))
                    last_move = self.stopping(last_move, vector)"""

            #else:
            if abs(self.new_orientation - self.curr_orientation) > 0.008:
                if self.new_orientation - self.curr_orientation > 0:
                    move.angular.z = 0.15
                    #last_move[2] = self.acceleratrion([0,0],last_move, axis=2, direction=1)
                else:
                    move.angular.z = -0.15
                    #last_move[2] = self.acceleratrion([0,0],last_move, axis=2, direction=-1)
            #else:
                #last_move = self.stopping(last_move, [0,0])
            #move.linear.x, move.linear.y, move.angular.z = last_move

            twist_pub.publish(move)
            rate.sleep()

    def acceleratrion(self, vector, last_move, axis, direction):
        last_velocity = last_move[axis]
        if vector[0] <= 0.3:
            max_velocity_straight = vector[0]
            min_velocity_straight = vector[0]
        else:
            max_velocity_straight = 0.3
            min_velocity_straight = 0.3
        if vector[1] <= 0.2:
            max_velocity_side = vector[1]
            min_velocity_side = vector[1]
        else:
            max_velocity_side = 0.2
            min_velocity_side = 0.2
        max_angular = 0.15
        min_angular = -0.15
        new_velocity = 0.0

        if last_velocity >= 0 and direction < 0:
            new_velocity = last_velocity - 0.08

        elif last_velocity < 0 and direction > 0:
            new_velocity = last_velocity + 0.08

        elif last_velocity >= 0 and direction > 0:
            new_velocity = (max_velocity_straight - last_velocity) / 10 + last_velocity
            if axis == 2:
                new_velocity = (max_angular - last_velocity) / 15 + last_velocity
            if axis == 1:
                new_velocity = (max_velocity_side - last_velocity) / 10 + last_velocity

        else:
            new_velocity = (min_velocity_straight - last_velocity) / 10 + last_velocity
            if axis == 2:
                new_velocity = (min_angular - last_velocity) / 15 + last_velocity
            if axis == 1:
                new_velocity = (min_velocity_side - last_velocity) / 10 + last_velocity
        return new_velocity

    def stopping(self,last_move, vector):
        if vector[0] != 0 and vector[1] != 0:
            if vector[0] > 0.1:
                last_move[0] -= 0.05
                """if last_move[axis] < 0:
                    last_move[axis] = 0"""
            if vector[0] < 0.1:
                last_move[0] += 0.05
                """if last_move[axis] > 0:
                    last_move[axis] = 0"""
            if vector[1] > 0.1:
                last_move[1] -= 0.05
            if vector[1] < 0.1:
                last_move[1] += 0.05
        else:
            if last_move[2] > 0.1:
                last_move[2] -= 0.05
            if last_move[2] < 0.1:
                last_move[2] += 0.05
        return last_move

    def set_speed(self, distance, last_move, vector):
        last_move_xy = np.array(last_move[:2])
        if distance < 2 and np.linalg.norm(last_move_xy) > 0.5:
            last_move_xy = last_move_xy - 0.005*vector
        elif distance < 1 and np.linalg.norm(last_move_xy) > 0.3:
            last_move_xy = last_move_xy - 0.008*vector
        elif distance < 0.5 and np.linalg.norm(last_move_xy) > 0.1:
            if np.allclose(np.sign(vector), np.sign(last_move_xy- 0.01*vector)):
                last_move_xy = last_move_xy - 0.01*vector
        elif distance > 0.15 and distance < 0.5 and np.linalg.norm(last_move_xy) < 0.1:
            last_move_xy = vector * 0.1
        else:
            if np.linalg.norm(last_move_xy) < 0.4:
                last_move_xy = last_move_xy + 0.08*vector
        last_move[:2] = last_move_xy
        return last_move

#MoveToPoint().move_to_point()

if __name__ == '__main__':
    try:
        rospy.init_node('move_to_point')
        MoveToPoint().move_to_point()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass






