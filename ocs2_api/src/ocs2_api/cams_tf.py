#!/usr/bin/env python

import rospy
import tf
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped
from tf.msg import tfMessage

class cams_tf:
    def __init__(self):
        self.curr_orientation_x = 0
        self.curr_orientation_y = 0
        self.curr_orientation_z = 0
        self.curr_orientation_w = 0

    def curr_state_callback(self,data):
        pose = data.pose[-1]
        orientation = pose.orientation
        self.curr_orientation_x = orientation.x
        self.curr_orientation_y = orientation.y
        self.curr_orientation_z = orientation.z
        self.curr_orientation_w = orientation.w

    def listener(self):
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.curr_state_callback)

    def tf_publisher(self):
        tf_cams_broadcaster = tf.TransformBroadcaster()
        #tf_cams_publisher = rospy.Publisher('tf_cams', tf, queue_size=10)
        rate = rospy.Rate(3)  # Adjust the rate as needed

        while not rospy.is_shutdown():
            self.listener()
            # Camera pose front right
            front_right_cam_xyz = [0.415, -0.037, -0.01]
            front_right_cam_rpy = [-1.355558, 0.353917, 0.554196]
            # Camera pose front left
            front_left_cam_xyz = [0.415, 0.037, -0.01]
            front_left_cam_rpy = [1.355558, 0.353917, -0.554196]
            # Camera pose right
            right_cam_xyz = [-0.125, -0.12, 0.035]
            right_cam_rpy = [0, 0.2, -1.5707963267948966]
            # Camera pose left
            left_cam_xyz = [-0.125, 0.12, 0.035]
            left_cam_rpy = [0, 0.2, 1.5707963267948966]
            # Camera pose back
            back_cam_xyz = [-0.425, 0, 0.01]
            back_cam_rpy = [0, 0.3, -3.141592653589793]
            camera_x = 1.0  # Replace with actual camera position X
            camera_y = 2.0  # Replace with actual camera position Y
            camera_z = 3.0  # Replace with actual camera position Z
            camera_roll = 0.1  # Replace with actual camera roll
            camera_pitch = 0.2  # Replace with actual camera pitch
            camera_yaw = 0.3  # Replace with actual camera yaw

            # Spot pose
            spot_x = self.curr_orientation_x
            spot_y = self.curr_orientation_y
            spot_z = self.curr_orientation_z
            spot_w = self.curr_orientation_w

            # Camera front right quaternions
            spot_quat = (0,0,0,spot_w)
            translation_fr = (spot_x + front_right_cam_xyz[0],
                              spot_y + front_right_cam_xyz[1],
                              spot_z + front_right_cam_xyz[2])
            quat_fr = quaternion_from_euler(front_right_cam_rpy[0],
                                            front_right_cam_rpy[1],
                                            front_right_cam_rpy[2])
            quat_fr = tf.transformations.quaternion_multiply(spot_quat,quat_fr)

            # Camera front left quaternions
            translation_fl = (spot_x + front_left_cam_xyz[0],
                              spot_y + front_left_cam_xyz[1],
                              spot_z + front_left_cam_xyz[2])
            quat_fl = quaternion_from_euler(front_left_cam_rpy[0],
                                            front_left_cam_rpy[1],
                                            front_left_cam_rpy[2])
            quat_fl = tf.transformations.quaternion_multiply(spot_quat, quat_fl)

            # Camera right quaternions
            translation_r = (spot_x + right_cam_xyz[0],
                              spot_y + right_cam_xyz[1],
                              spot_z + right_cam_xyz[2])
            quat_r = quaternion_from_euler(right_cam_rpy[0],
                                            right_cam_rpy[1],
                                            right_cam_rpy[2])
            quat_r = tf.transformations.quaternion_multiply(spot_quat, quat_r)

            # Camera left quaternions
            translation_l = (spot_x + left_cam_xyz[0],
                             spot_y + left_cam_xyz[1],
                             spot_z + left_cam_xyz[2])
            quat_l = quaternion_from_euler(left_cam_rpy[0],
                                           left_cam_rpy[1],
                                           left_cam_rpy[2])
            quat_l = tf.transformations.quaternion_multiply(spot_quat, quat_l)

            # Camera back quaternions
            translation_b = (spot_x + back_cam_xyz[0],
                             spot_y + back_cam_xyz[1],
                             spot_z + back_cam_xyz[2])
            quat_b = quaternion_from_euler(back_cam_rpy[0],
                                           back_cam_rpy[1],
                                           back_cam_rpy[2])
            quat_b = tf.transformations.quaternion_multiply(spot_quat, quat_b)

            # Broadcast the transformation
            print(quat_fr)
            tf_cams_broadcaster.sendTransform(translation_fr, quat_fr,
                                         rospy.Time.now(), "front_right_camera_frame", "world")
            tf_cams_broadcaster.sendTransform(translation_fl, quat_fl,
                                         rospy.Time.now(), "front_left_camera_frame", "world")
            tf_cams_broadcaster.sendTransform(translation_r, quat_r,
                                         rospy.Time.now(), "right_camera_frame", "world")
            tf_cams_broadcaster.sendTransform(translation_l, quat_l,
                                         rospy.Time.now(), "left_camera_frame", "world")
            tf_cams_broadcaster.sendTransform(translation_b, quat_b,
                                         rospy.Time.now(), "back_camera_frame", "world")
            #print(tf_cams_broadcaster)
            print("sent")
            #tf_cams_publisher.publish(tf_cams_broadcaster)


            rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('cameras_transformation')
        cams_tf().tf_publisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
