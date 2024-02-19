#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
import numpy as np
import tf
from std_msgs.msg import Bool

class MoveToPoint:
    def __init__(self):
        self.new_x = 0
        self.new_y = 0
        self.curr_position_x = 0
        self.curr_position_y = 0
        self.curr_orientation = 0
        self.got_command2 = False
        self.rotate = True
        self.detect_obstacle = False
        self.last_x = 0
        self.last_y = 0
        self.last_ang_z = 0
    def new_state_callback(self,data):
        point = data.point
        self.new_x = point.x
        self.new_y = point.y
        self.got_command2 = True
        self.rotate = True
        self.last_x = 0
        self.last_y = 0
        self.last_ang_z = 0
    def curr_state_callback(self,data):
        pose = data.pose[-1]
        self.curr_position_x = pose.position.x
        self.curr_position_y = pose.position.y
        self.curr_orientation_x = pose.orientation.x
        self.curr_orientation_y = pose.orientation.y
        self.curr_orientation_z = pose.orientation.z
        self.curr_orientation_w = pose.orientation.w

    def new_request_callback(self,data):
        if data is not None:
            self.got_command2 = False

    def detect_obstacle_callback(self,data):
        self.detect_obstacle = data.data


    def listener(self):
        rospy.Subscriber("/clicked_point", PointStamped, self.new_state_callback)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.curr_state_callback)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.new_request_callback)
        rospy.Subscriber("/slow_down", Bool, self.detect_obstacle_callback)

    def coord_transformation(self,theta,x,y):
        T = np.array([
            [np.cos(theta), np.sin(theta)],
             [-np.sin(theta), np.cos(theta)]
        ])
        vector = T @ np.array([x, y])
        return vector

    def new_orientation(self):
        spot_to_point = [self.new_x - self.curr_position_x, self.new_y - self.curr_position_y]
        orientation_to_point = np.arctan2(spot_to_point[1], spot_to_point[0]) # /np.pi
        return spot_to_point, orientation_to_point

    def euler_from_quaternion(self):
        euler = tf.transformations.euler_from_quaternion([self.curr_orientation_x, self.curr_orientation_y,
                                                         self.curr_orientation_z, self.curr_orientation_w])
        return euler[-1]

    def move_to_point(self):
        twist_pub = rospy.Publisher('twist_cmd', Twist, queue_size=10)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.listener()
            if self.got_command2:
                move = Twist()
                spot_to_point, orientation_to_point = self.new_orientation()
                norm = np.linalg.norm(spot_to_point)
                current_angle = self.euler_from_quaternion()
                vector = self.coord_transformation(current_angle, spot_to_point[0], spot_to_point[1])
                max_speed = 0.6
                if self.detect_obstacle:
                    max_speed /= 2
                if abs(orientation_to_point - current_angle) > 0.5 and np.linalg.norm(vector) > 0.25:
                    if orientation_to_point - current_angle > 0.5:
                        self.last_ang_z = self.acceleration(self.last_ang_z,0.5)
                    if orientation_to_point - current_angle < -0.5:
                        self.last_ang_z = self.acceleration(self.last_ang_z, -0.5)

                elif abs(orientation_to_point - current_angle) <= 0.5 and abs(orientation_to_point - current_angle) > 0.1 and np.linalg.norm(vector) > 0.25:
                    self.last_ang_z = self.acceleration(self.last_ang_z, orientation_to_point - current_angle)

                else:
                    self.rotate = False
                    self.last_ang_z = 0
                if not self.rotate:
                    norm_vector = vector / norm
                    if norm > 2:
                        self.last_x = self.acceleration(self.last_x, max_speed * norm_vector[0])
                        self.last_y = self.acceleration(self.last_y, (max_speed / 2) * norm_vector[1])
                    elif norm > 1:
                        self.last_x = self.acceleration(self.last_x, 0.3 * norm_vector[0])
                        self.last_y = self.acceleration(self.last_y, 0.2 * norm_vector[1])
                    elif norm > 0.05:
                        if 0.15 * vector[0] < 0.05:
                            self.last_x = 0.02 * norm_vector[0]
                        else:
                            self.last_x = self.acceleration(self.last_x,0.15 * vector[0])
                        if 0.1 * vector[1] < 0.05:
                            self.last_y = 0.02 * norm_vector[1]
                        else:
                            self.last_y = self.acceleration(self.last_y, 0.08 * vector[1])
                    else:
                        self.got_command2 = False
                        self.last_x = 0
                        self.last_y = 0
                move.linear.x, move.linear.y, move.angular.z = self.last_x, self.last_y, self.last_ang_z
                twist_pub.publish(move)
            rate.sleep()

    def acceleration(self, last_move, desired_move):
        new_move = last_move
        if last_move < desired_move:
            new_move = last_move + 0.25*(desired_move-last_move)
        if last_move > desired_move:
            new_move = last_move - 0.25*(last_move-desired_move)
        return new_move
