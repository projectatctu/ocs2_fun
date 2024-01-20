#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
import numpy as np

class MoveToPoint:
    def __init__(self):
        self.new_x = 0
        self.new_y = 0
        self.curr_position_x = 0
        self.curr_position_y = 0
        self.curr_orientation = 0
        self.got_command2 = False
        #rospy.init_node('tu')
    def new_state_callback(self,data):
        point = data.point
        self.new_x = point.x
        self.new_y = point.y
        self.got_command2 = True
    def curr_state_callback(self,data):
        pose = data.pose[-1]
        self.curr_position_x = pose.position.x
        self.curr_position_y = pose.position.y
        self.curr_orientation = pose.orientation.z

    def new_request_callback(self,data):
        if data is not None:
            self.got_command2 = False

    def listener(self):
        rospy.Subscriber("/clicked_point", PointStamped, self.new_state_callback)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.curr_state_callback)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.new_request_callback)

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
            if not self.got_command2:
                self.new_x = self.curr_position_x
                self.new_y = self.curr_position_y
                #self.got_command = True
                #continue
            new_orientation = np.arctan2(self.new_y-self.curr_position_y, self.new_x-self.curr_position_x) / np.pi
            move = Twist()
            curr_point = np.array([self.curr_position_x, self.curr_position_y])
            end_point = np.array([self.new_x, self.new_y])
            vector = end_point-curr_point

            if np.linalg.norm(vector) > 1:
                vector = self.coord_transformation(self.curr_orientation * np.pi, vector[0], vector[1])
                vector = (vector/np.linalg.norm(vector))
                move.linear.x = vector[0] * 0.3
                move.linear.y = vector[1] * 0.2
                if abs(new_orientation - self.curr_orientation) > 0.05:
                    if new_orientation - self.curr_orientation > 0:
                        move.angular.z = 0.15
                    else:
                        move.angular.z = - 0.15
            else:
                vector = self.coord_transformation(self.curr_orientation * np.pi, vector[0], vector[1])
                if np.linalg.norm(vector) > 0.5:
                    move.linear.x = vector[0] * 0.3 * np.linalg.norm(vector)
                    move.linear.y = vector[1] * 0.2 * np.linalg.norm(vector)
                elif np.linalg.norm(vector) > 0.2:
                    move.linear.x = vector[0] * np.linalg.norm(vector)
                    move.linear.y = vector[1] * np.linalg.norm(vector)
                elif np.linalg.norm(vector) > 0.01:
                    move.linear.x = vector[0]
                    move.linear.y = vector[1]
                else:
                    self.got_command2 = False
            """else:
                if np.linalg.norm(vector) > 0.05:
                    move.linear.x = vector[0] * 0.4 #* np.linalg.norm(vector)
                    move.linear.y = vector[1] * 0.3 #* np.linalg.norm(vector)
                else:
                    self.got_command = False"""

            if self.got_command2:
                #print("publishing")
                twist_pub.publish(move)
            rate.sleep()
#MoveToPoint().move_to_point()

"""if __name__ == '__main__':
    try:
        rospy.init_node('move_to_point')
        MoveToPoint().move_to_point()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass"""






