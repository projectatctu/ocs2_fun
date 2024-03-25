#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class ArmCamera:
    def __init__(self):
        self.img = None
        self.bridge = CvBridge()
        self.delay = 5000 # 5000 ms

    def img_callback(self, data):
        self.img = data
        self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
    def listener(self):
        rospy.Subscriber("/camera/arm/rgb/image_raw", Image, self.img_callback)

    def main(self):
        rate = rospy.Rate(1)
        ret = 1
        while not rospy.is_shutdown():
            self.listener()
            if self.img is not None:
                cv2.imshow("Arm vision", self.img)
                cv2.waitKey(self.delay)
                cv2.destroyAllWindows()
                ret = 0
                break
            rate.sleep()
        return ret

if __name__ == '__main__':
    rospy.init_node('Arm_vision')
    ret = ArmCamera().main()
    if ret:
        rospy.spin()