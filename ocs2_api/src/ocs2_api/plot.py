#!/usr/bin/env python

import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import PointCloud2
import numpy as np
from sensor_msgs import point_cloud2
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from gazebo_msgs.msg import ModelStates
import tf




class DepthCamera_img:
    def __init__(self):
        self.img = None
        #rospy.init_node('tu')
    def image_callback(self,data):
        self.img = data

    def listener(self):
        rospy.Subscriber("/completed_map", PointCloud2, self.image_callback)

    def show_image(self):
        rate = rospy.Rate(1)
        fig = plt.figure(figsize=(20,10))

        while not rospy.is_shutdown():
            plt.clf()
            self.listener()
            cloud_img = self.img

            if cloud_img is not None:
                cloud_points_img = list(point_cloud2.read_points(cloud_img, skip_nans=True, field_names=("x", "y", "z")))

                ax = fig.add_subplot(projection='3d')
                point_array_img = np.array(cloud_points_img)
                ax.set_xlabel('X')
                ax.set_ylabel('Y')
                ax.set_zlabel('Z')
                plt.show()
                plt.pause(1)
                """points_array_front_right = np.array(cloud_points_front_right)
                points_array_front_left = np.array(cloud_points_front_left)
                points_array_right = np.array(cloud_points_right)
                points_array_left = np.array(cloud_points_left)
                points_array_back = np.array(cloud_points_back)

                points_array_front_right = points_array_front_right.dot(R_front_right.T)
                points_array_front_left = points_array_front_left.dot(R_front_left.T)
                points_array_right = points_array_right.dot(R_right)
                points_array_left = points_array_left.dot(R_left)
                points_array_back = points_array_back.dot(R_back)
                points_array_back = self.surface_transform(points_array_back,trans_b)"""

                """ax.scatter(points_array_front_left[:, 0], points_array_front_left[:, 1], points_array_front_left[:, 2])
                ax.set_xlabel('X')
                ax.set_ylabel('Y')
                ax.set_zlabel('Z')
                ax.set_title('Front left camera')

                ax = fig.add_subplot(2, 3, 2, projection='3d')
                ax.scatter(points_array_front_right[:, 0], points_array_front_right[:, 1], points_array_front_right[:,2])
                ax.set_xlabel('X')
                ax.set_ylabel('Y')
                ax.set_zlabel('Z')
                ax.set_title('Front right camera')

                ax = fig.add_subplot(2, 3, 3, projection='3d')
                ax.scatter(points_array_right[:, 0], points_array_right[:, 1], points_array_right[:, 2])
                ax.set_xlabel('X')
                ax.set_ylabel('Y')
                ax.set_zlabel('Z')
                ax.set_title('Right camera')

                ax = fig.add_subplot(2, 3, 4, projection='3d')
                ax.scatter(points_array_left[:, 0], points_array_left[:, 1], points_array_left[:, 2])
                ax.set_xlabel('X')
                ax.set_ylabel('Y')
                ax.set_zlabel('Z')
                ax.set_title('Left camera')

                ax = fig.add_subplot(2, 3, 5, projection='3d')
                ax.scatter(points_array_back[:, 0], points_array_back[:, 1], points_array_back[:, 2])
                ax.set_xlabel('X')
                ax.set_ylabel('Y')
                ax.set_zlabel('Z')
                ax.set_title('Back camera')
                plt.show(block=False)
                plt.show()
                plt.waitkey()
                #plt.pause(1)"""
            rate.sleep()



if __name__ == '__main__':
    try:
        rospy.init_node('ploting')
        DepthCamera_img().show_image()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass





