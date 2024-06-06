#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from lane_detection_utils import *
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import numpy as np


class StartSignalDetector:
    def __init__(self):
        self.node_name = "start_signal"
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(1)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)
        self.signal_pub = rospy.Publisher('/start_signal', Bool, queue_size=1)
        self.debug = True
        self.roi = (560, 315, 22, 20)
        
        self.start_signal = False

    def callback(self, data):
        try:
            # if self.start_signal:
            #     self.signal_pub.publish(self.start_signal)
            #     return

            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            height, width = cv_image.shape[:2]
    
            hls_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HLS)
            processed_image = self.set_roi(hls_image, self.roi)

            # if self.debug:      
            #     self.roi = cv2.selectROI("Select ROI", cv_image, fromCenter=False, showCrosshair=True)
            #     cv2.destroyWindow("Select ROI")
            #     print(f"Selected ROI: {self.roi}")

            if self.debug:  
                def get_pixel_value(event, x, y, flags, param):
                    if event == cv2.EVENT_LBUTTONDOWN:
                        value = cv_image[y, x]
                        print("클릭한 위치 (x={}, y={})".format(x, y))
                        print("색상 값:", value)
                cv2.imshow("processed_image", cv_image)
                cv2.setMouseCallback('processed_image', get_pixel_value)
                cv2.waitKey(1)

            lower_green = np.array([40, 210, 245])
            upper_green = np.array([55, 225, 255])
            mask = cv2.inRange(processed_image, lower_green, upper_green)

            green_pixels = np.sum(mask == 255)
            total_pixels = mask.size
            green_ratio = green_pixels / total_pixels

            signal = green_ratio > 0.35
            if signal:
                self.start_signal = True

            # self.signal_pub.publish(self.start_signal)


        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return

    def set_roi(self, image, roi):
        x, y, w, h = roi
        return image[int(y):int(y+h), int(x):int(x+w)]

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        start = StartSignalDetector()
        start.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("start signal node terminated.")
