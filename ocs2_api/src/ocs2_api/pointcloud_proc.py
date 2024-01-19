#!/usr/bin/env python

import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import PointCloud2
import get_new_point
import get_curr_point
import numpy as np
from sensor_msgs import point_cloud2
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from gazebo_msgs.msg import ModelStates
import tf




class DepthCamera_img:
    def __init__(self):
        self.img_front_right = None
        self.img_front_left = None
        self.img_right = None
        self.img_left = None
        self.img_back = None
        self.curr_orientation_x = 0
        self.curr_orientation_y = 0
        self.curr_orientation_z = 0
        self.curr_orientation_w = 0
        self.front_right_cam_orientation = None
        self.front_left_cam_orientation = None
        #rospy.init_node('tu')
    def image_callback_front_right(self,data):
        self.img_front_right = data

    def image_callback_front_left(self,data):
        self.img_front_left = data

    def image_callback_right(self,data):
        self.img_right = data

    def image_callback_left(self,data):
        self.img_left = data

    def image_callback_back(self,data):
        self.img_back = data

    def curr_state_callback(self, data):
        pose = data.pose[-1]
        orientation = pose.orientation
        self.curr_orientation_x = orientation.x
        self.curr_orientation_y = orientation.y
        self.curr_orientation_z = orientation.z
        self.curr_orientation_w = orientation.w

    def listener(self):
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.curr_state_callback)
        rospy.Subscriber("/camera_front_right/depth/points_downsampled", PointCloud2, self.image_callback_front_right)
        rospy.Subscriber("/camera_front_left/depth/points_downsampled", PointCloud2, self.image_callback_front_left)
        rospy.Subscriber("/camera_right/depth/points_downsampled", PointCloud2, self.image_callback_right)
        rospy.Subscriber("/camera_left/depth/points_downsampled", PointCloud2, self.image_callback_left)
        rospy.Subscriber("/camera_back/depth/points_downsampled", PointCloud2, self.image_callback_back)



    def rot_x(self, angle):
        rx = np.array([
            [1,0,0],
            [0,np.cos(angle),-np.sin(angle)],
            [0,np.sin(angle),np.cos(angle)]
        ])
        return rx

    def rot_y(self, angle):
        ry = np.array([
            [np.cos(angle),0,np.sin(angle)],
            [0,1,0],
            [-np.sin(angle),0,np.cos(angle)]
        ])
        return ry

    def rot_z(self, angle):
        rz = np.array([
            [np.cos(angle),-np.sin(angle),0],
            [np.sin(angle),np.cos(angle),0],
            [0,0,1]
        ])
        return rz

    def front_right_orientation(self,x,y,z,w):
        omega = np.array([
            [0, -z, y],
            [z, 0, -x],
            [-y, x, 0]
        ])
        R = (np.identity(3) + np.sin(w) * omega +
                (1 - np.cos(w)) * (omega @ omega))
        return R

    def front_left_orientation(self,x,y,z,w):
        omega = np.array([
            [0, -z, y],
            [z, 0, -x],
            [-y, x, 0]
        ])
        R = (np.identity(3) + np.sin(w) * omega +
                (1 - np.cos(w)) * (omega @ omega))
        return R

    def spot_orientation(self):
        omega = np.array([
            [0, -self.curr_orientation_z, self.curr_orientation_y],
            [self.curr_orientation_z, 0, -self.curr_orientation_x],
            [-self.curr_orientation_y, self.curr_orientation_x, 0]
        ])
        R = (np.identity(3) + np.sin(self.curr_orientation_w) * omega +
             (1 - np.cos(self.curr_orientation_w)) * (omega @ omega))
        return R

    def exp(self,rot_vec):
        theta = (rot_vec[0] ** 2 + rot_vec[1] ** 2 + rot_vec[2] ** 2) ** (1 / 2)
        if theta != 0:
            v_nor = rot_vec / theta
        else:
            v_nor = np.array([0, 0, 0])
        skew_mat = np.array([
            [0, -v_nor[2], v_nor[1]],
            [v_nor[2], 0, -v_nor[0]],
            [-v_nor[1], v_nor[0], 0]
        ])
        skew_mat2 = skew_mat @ skew_mat
        R = np.identity(3) + np.sin(theta) * skew_mat + (1 - np.cos(theta)) * skew_mat2
        return R
    def rot_quaternions(self, rot):
        theta = 2*np.arccos(rot[3])
        R_exp = self.exp(theta*np.array(rot[:3])/np.sqrt(np.sum(np.array(rot[:3])**2)))
        return R_exp

    def surface_transform(self, points,transl):
        new_trans = np.ones(np.shape(points[:,:2]))
        new_trans[:,0] *=transl[0]
        new_trans[:,1] *=transl[1]
        alpha = np.arctan2(points[:,1]-new_trans[:,1],points[:,0]-new_trans[:,0])
        norm = np.linalg.norm(points[:,:2],axis=1)
        points[:,2] = alpha*norm-points[:,2]
        return points


    def show_image(self):
        rate = rospy.Rate(1)
        fig = plt.figure(figsize=(20,10))
        tf_listener = tf.TransformListener()
        rot_fr = None
        rot_fl = None
        rot_r = None
        rot_l = None
        rot_b = None
        while not rospy.is_shutdown():
            plt.clf()
            self.listener()
            #------------------------------------------front right camera---------------------------------------------
            try:
                #  '/base','/camera_link_front_right'
                tf_listener.waitForTransform('/world','/front_right_camera_frame',  rospy.Time(), rospy.Duration(1.0))
                (trans_fr, rot_fr) = tf_listener.lookupTransform('/front_right_camera_frame','/world',  rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("Failed to get fr transformation")
            #-----------------------------------------front left camera-----------------------------------------------
            try:
                tf_listener.waitForTransform('/world', '/front_left_camera_frame', rospy.Time(), rospy.Duration(1.0))
                (trans_fl, rot_fl) = tf_listener.lookupTransform('/world', '/front_left_camera_frame', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("Failed to get fl transformation")

            # ------------------------------------------right camera---------------------------------------------
            try:
                #  '/base','/camera_link_front_right'
                tf_listener.waitForTransform('/world','/right_camera_frame',  rospy.Time(),
                                             rospy.Duration(1.0))
                (trans_r, rot_r) = tf_listener.lookupTransform('/right_camera_frame', '/world',
                                                                 rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("Failed to get fr transformation")
            # -----------------------------------------left camera-----------------------------------------------
            try:
                tf_listener.waitForTransform('/world', '/left_camera_frame', rospy.Time(),
                                             rospy.Duration(1.0))
                (trans_l, rot_l) = tf_listener.lookupTransform('/world', '/left_camera_frame',
                                                                 rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("Failed to get fl transformation")
            # -----------------------------------------back camera-----------------------------------------------
            try:
                tf_listener.waitForTransform('/world', '/back_camera_frame', rospy.Time(),
                                             rospy.Duration(1.0))
                (trans_b, rot_b) = tf_listener.lookupTransform('/world', '/back_camera_frame',
                                                                 rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("Failed to get fl transformation")

            if rot_fr is not None:
                #R_front_right = self.rot_quaternions(rot_fr)
                R_front_right = self.front_right_orientation(rot_fr[0], rot_fr[1], rot_fr[2], rot_fr[3])
            if rot_fl is not None:
                #R_front_left = self.rot_quaternions(rot_fl)
                R_front_left = self.front_left_orientation(rot_fl[0], rot_fl[1], rot_fl[2], rot_fl[3])
            if rot_r is not None:
                R_right = self.rot_quaternions(rot_r)
            if rot_l is not None:
                R_left = self.rot_quaternions(rot_l)
            if rot_b is not None:
                R_back = self.rot_quaternions(rot_b)
            cloud_front_right = self.img_front_right
            cloud_front_left = self.img_front_left
            cloud_right = self.img_right
            cloud_left = self.img_left
            cloud_back = self.img_back

            if (cloud_front_right is not None and cloud_front_left and cloud_right is not None
                    and cloud_left is not None and cloud_back is not None
                    and rot_fr is not None and rot_fl is not None and rot_r is not None
                    and rot_l is not None and rot_b is not None):
                cloud_points_front_right = list(point_cloud2.read_points(cloud_front_right, skip_nans=True, field_names=("x", "y", "z")))
                cloud_points_front_left = list(point_cloud2.read_points(cloud_front_left, skip_nans=True, field_names=("x", "y", "z")))
                cloud_points_right = list(point_cloud2.read_points(cloud_right, skip_nans=True, field_names=("x", "y", "z")))
                cloud_points_left = list(point_cloud2.read_points(cloud_left, skip_nans=True, field_names=("x", "y", "z")))
                cloud_points_back = list(point_cloud2.read_points(cloud_back, skip_nans=True, field_names=("x", "y", "z")))

                ax = fig.add_subplot(2,3,1,projection='3d')
                points_array_front_right = np.array(cloud_points_front_right)
                points_array_front_left = np.array(cloud_points_front_left)
                points_array_right = np.array(cloud_points_right)
                points_array_left = np.array(cloud_points_left)
                points_array_back = np.array(cloud_points_back)

                points_array_front_right = points_array_front_right.dot(R_front_right.T)
                points_array_front_left = points_array_front_left.dot(R_front_left.T)
                points_array_right = points_array_right.dot(R_right)
                points_array_left = points_array_left.dot(R_left)
                points_array_back = points_array_back.dot(R_back)
                points_array_back = self.surface_transform(points_array_back,trans_b)

                ax.scatter(points_array_front_left[:, 0], points_array_front_left[:, 1], points_array_front_left[:, 2])
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
                #plt.pause(1)
            rate.sleep()



if __name__ == '__main__':
    try:
        rospy.init_node('pointcloud_cam')
        DepthCamera_img().show_image()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass





