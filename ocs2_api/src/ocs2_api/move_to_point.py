#!/usr/bin/env python
import sys
sys.path.append("/opt/ros/noetic/lib/python3/dist-packages")

import rospy
from geometry_msgs.msg import PointStamped
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
import get_new_point
import get_curr_point
import numpy as np

class MoveToPoint:
    def __init__(self):
        self.new_x = 0
        self.new_y = 0
        self.curr_position_x = 0
        self.curr_position_y = 0
        self.curr_orientation = 0
        #rospy.init_node('tu')
    def new_state_callback(self,data):
        point = data.point
        self.new_x = point.x
        self.new_y = point.y
    def curr_state_callback(self,data):
        pose = data.pose[-1]
        self.curr_position_x = pose.position.x
        self.curr_position_y = pose.position.y
        self.curr_orientation = pose.orientation.z

    def listener(self):
        rospy.Subscriber("/clicked_point", PointStamped, self.new_state_callback)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.curr_state_callback)

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
            new_orientation = np.arctan2(self.new_y-self.curr_position_y, self.new_x-self.curr_position_x) / np.pi
            self.listener()
            if self.new_x == 0 and self.new_y == 0:
                self.new_x = self.curr_position_x
                self.new_y = self.curr_position_y
            move = Twist()
            curr_point = np.array([self.curr_position_x, self.curr_position_y])
            end_point = np.array([self.new_x, self.new_y])
            vector = end_point-curr_point

            if np.linalg.norm(end_point - curr_point) > 0.1:
                vector = self.coord_transformation(self.curr_orientation * np.pi, vector[0], vector[1])
                vector = (vector/np.linalg.norm(vector))
                move.linear.x = vector[0] * 0.3
                move.linear.y = vector[1] * 0.2
                if abs(new_orientation - self.curr_orientation) > 0.05:
                    if new_orientation - self.curr_orientation > 0:
                        move.angular.z = 0.15
                    else:
                        move.angular.z = - 0.15

            twist_pub.publish(move)
            rate.sleep()
#MoveToPoint().move_to_point()

if __name__ == '__main__':
    try:
        rospy.init_node('move_to_point')
        MoveToPoint().move_to_point()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass






