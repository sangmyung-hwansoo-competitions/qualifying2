#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import time
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from perception_utils import *

class StartSignalDetector:
    def __init__(self):
        self.node_name = "start_signal"
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(2)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        self.signal_pub = rospy.Publisher('/start_signal', Bool, queue_size=1)

        self.is_image = False
        self.image_data = None
        self.roi = (560, 305, 23, 23)
        # self.roi = (600, 200, 30, 30)

        self.start_signal = False
        
        self.debug = True

    def image_callback(self, msg):
        if self.start_signal:
            return

        self.is_image = True
        self.image_data = msg

    def set_roi(self, image, roi):
        x, y, w, h = roi
        return image[int(y):int(y+h), int(x):int(x+w)]

    def pub_signal(self):
        if self.start_signal:
            self.signal_pub.publish(self.start_signal)
            return

        if self.is_image:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(self.image_data, "bgr8")

                # if self.debug:      
                #     self.roi = cv2.selectROI("Select ROI", cv_image, fromCenter=False, showCrosshair=True)
                #     cv2.destroyWindow("Select ROI")
                #     print(f"Selected ROI: {self.roi}")

                processed_image = self.set_roi(cv_image, self.roi)

                lower_green = np.array([150, 245, 200])
                upper_green = np.array([180, 255, 240])
                mask = cv2.inRange(processed_image, lower_green, upper_green)

                # if self.debug:  
                #     def get_pixel_value(event, x, y, flags, param):
                #         if event == cv2.EVENT_LBUTTONDOWN:
                #             value = mask[y, x]
                #             print("클릭한 위치 (x={}, y={})".format(x, y))
                #             print("색상 값:", value)
                #     cv2.imshow("processed_image", mask)
                #     cv2.setMouseCallback('processed_image', get_pixel_value)
                #     cv2.waitKey(1)

                green_pixels = np.sum(mask == 255)
                total_pixels = mask.size
                green_ratio = green_pixels / total_pixels
                signal = green_ratio > 0.4
                # rospy.loginfo(f"ratio: {green_ratio}")

                if signal:
                    # rospy.loginfo(f"START")
                    self.start_signal = True

                self.signal_pub.publish(self.start_signal)

            except CvBridgeError as e:
                rospy.logerr(f"CvBridge Error: {e}")
                return

    def run(self):
        while not rospy.is_shutdown():
            self.pub_signal()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        start = StartSignalDetector()
        start.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("start signal node terminated.")
