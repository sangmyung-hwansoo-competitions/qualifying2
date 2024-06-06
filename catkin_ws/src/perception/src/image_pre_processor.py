#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from perception_utils import *
from moving_average_filter import MovingAverageFilter, FilteringMode

class ImagePreProcessor:
    def __init__(self):
        self.node_name = "image_processor"
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(20)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        self.image_pub = rospy.Publisher('/image_processor/image_processed', Image, queue_size=1)

        self.is_image = False
        self.image_data = None
        self.roi = (0, 310, 800, 235) # 235, 800, 3

        self.debug = True

    def image_callback(self, msg):
        self.is_image = True
        self.image_data = msg

    def set_roi(self, image, roi):
        x, y, w, h = roi
        return image[int(y):int(y+h), int(x):int(x+w)]

    def masked_roi(self, image):
        x = int(image.shape[1])
        y = int(image.shape[0])

        # 한 붓 그리기
        # https://moon-coco.tistory.com/entry/OpenCV%EC%B0%A8%EC%84%A0-%EC%9D%B8%EC%8B%9D
        _shape = np.array(
            
            [[int(0*x), int(1*y)], 
            [int(0*x), int(0*y)], 
            [int(0.4*x), int(0*y)], 
            [int(0.4*x), int(1*y)], 
            [int(0.6*x), int(1*y)], 
            [int(0.6*x), int(0*y)],
            [int(1*x), int(0*y)], 
            [int(1*x), int(1*y)]])

        mask = np.zeros_like(image)

        if len(image.shape) > 2:
            channel_count = image.shape[2]
            ignore_mask_color = (255,) * channel_count
        else:
            ignore_mask_color = 255

        cv2.fillPoly(mask, np.int32([_shape]), ignore_mask_color)
        masked_image = cv2.bitwise_and(image, mask)

        return masked_image

    def pub_image(self):
        if self.is_image:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(self.image_data, "bgr8")

                # processed_image = self.set_roi(cv_image, self.roi)
                processed_image = bev_transform(cv_image)
                processed_image = adjust_brightness_contrast(processed_image, brightness=-30, contrast=30)
                processed_image = apply_clahe(processed_image)
                processed_image = adjust_gamma(processed_image, gamma=1.2)
                
                # processed_image = cv2.cvtColor(processed_image, cv2.COLOR_BGR2HLS)

                blur_img = gaussian_blur(processed_image, 5) # Blur 효과      
                gray_img = grayscale(blur_img) # 흑백이미지로 변환
                canny_img = canny(gray_img, 45, 210) # Canny edge 알고리즘

                # if self.debug:      
                    # self.roi = cv2.selectROI("Select ROI", cv_image, fromCenter=False, showCrosshair=True)
                    # cv2.destroyWindow("Select ROI")
                    # print(f"Selected ROI: {self.roi}")
                    
                    # def get_pixel_value(event, x, y, flags, param):
                    #     if event == cv2.EVENT_LBUTTONDOWN:
                    #         value = canny_img[y, x]
                    #         print("클릭한 위치 (x={}, y={})".format(x, y))
                    #         print("색상 값:", value)
                    # cv2.imshow("processed_image", canny_img)
                    # cv2.setMouseCallback('processed_image', get_pixel_value)
                    # cv2.waitKey(1)


                processed_msg = self.bridge.cv2_to_imgmsg(canny_img, encoding='mono8')
                self.image_pub.publish(processed_msg)
              

            except CvBridgeError as e:
                rospy.logerr(f"CvBridge Error: {e}")
                return

    def run(self):
        while not rospy.is_shutdown():
            self.pub_image()
            self.rate.sleep()


if __name__ == '__main__':
    try:
        lane_detector = ImagePreProcessor()
        lane_detector.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("lane detector node terminated.")
