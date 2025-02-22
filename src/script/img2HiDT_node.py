#!/usr/bin/env python3

import roslib
roslib.load_manifest('nav_cloning')
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from HiDT_transformer import *

class img2HiDT_node:
    def __init__(self):
        rospy.init_node('img2HiDT_node', anonymous=True)
        self.bridge = CvBridge()
        self.H_trans = HiDT_transformer()

        ### zero array ###
        self.cv_image = np.zeros((480,640,3), np.uint8)
        self.cv_left_image = np.zeros((480,640,3), np.uint8)
        self.cv_right_image = np.zeros((480,640,3), np.uint8)
        self.HiDT_image = self.bridge.cv2_to_imgmsg(self.cv_image, encoding="rgb8")
        self.HiDT_left_image = self.bridge.cv2_to_imgmsg(self.cv_left_image, encoding="rgb8")
        self.HiDT_right_image = self.bridge.cv2_to_imgmsg(self.cv_right_image, encoding="rgb8")

        ### Subscriber ###
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback_camera)
        self.image_left_sub = rospy.Subscriber("/camera_left/rgb/image_raw", Image, self.callback_left_camera)
        self.image_right_sub = rospy.Subscriber("/camera_right/rgb/image_raw", Image, self.callback_right_camera)

        ### Publisher ###
        self.image_pub = rospy.Publisher('/hidt_camera/rgb/image_raw', Image, queue_size=1)
        self.image_left_pub = rospy.Publisher('/hidt_camera_left/rgb/image_raw', Image, queue_size=1)
        self.image_right_pub = rospy.Publisher('/hidt_camera_right/rgb/image_raw', Image, queue_size=1)

    def callback_camera(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.HiDT_image = img2HiDT(self.cv_image)
            #self.image_pub.publish(self.HiDT_image)
        except CvBridgeError as e:
            print(e)

    def callback_left_camera(self, data):
        try:
            self.cv_left_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.HiDT_left_image = img2HiDT(self.cv_left_image)
            #self.image_left_pub.publish(self.HiDT_left_image)
        except CvBridgeError as e:
            print(e)

    def callback_right_camera(self, data):
        try:
            self.cv_right_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.HiDT_right_image = img2HiDT(self.cv_right_image)
            #self.image_right_pub.publish(self.HiDT_right_image)
        except CvBridgeError as e:
            print(e)

    def img2HiDT(self, image):
        #trans_image = np.zeros((480,640,3), np.uint8)

        if self.image.size != 640 * 480 * 3:
            return

        trans_image = self.H_trans.HiDT(image)

        # numpy配列をROSのImageメッセージに変換
        trans_image = self.bridge.cv2_to_imgmsg(trans_image, encoding="rgb8")

        return trans_image

    def loop(self):
        self.image_pub.publish(self.HiDT_image)
        self.image_left_pub.publish(self.HiDT_left_image)
        self.image_right_pub.publish(self.HiDT_right_image)


if __name__ == '__main__':
    node = img2HiDT_node()

    DURATION = 0.2
    r = rospy.Rate(1 / DURATION)
    while not rospy.is_shutdown():
        node.loop()
        r.sleep()
