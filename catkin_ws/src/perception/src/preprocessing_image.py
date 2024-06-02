#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import argparse
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError


class ImagePreProcessor:
    def __init__(self, brightness, contrast, saturation):
        self.node_name = "image_PreProcessor"
        rospy.init_node(self.node_name, anonymous=True)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)
        self.image_pub = rospy.Publisher("/usb_cam/image_processed", Image, queue_size=1)

        self.brightness = brightness
        self.contrast = contrast
        self.saturation = saturation

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return

        processed_image = self.process_image(cv_image)

        try:
            image_msg = self.bridge.cv2_to_imgmsg(processed_image)
            image_msg.header.stamp = rospy.Time.now()
            image_msg.header.frame_id = "camera"
            self.image_pub.publish(image_msg)
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")

    def process_image(self, image):
        # 밝기 및 대비 조절
        adjusted_image = cv2.convertScaleAbs(image, alpha=self.contrast, beta=self.brightness)

        # 채도 조절
        hsv_image = cv2.cvtColor(adjusted_image, cv2.COLOR_BGR2HSV)
        hsv_image[:, :, 1] = cv2.multiply(hsv_image[:, :, 1], self.saturation)
        saturated_image = cv2.cvtColor(hsv_image, cv2.COLOR_HSV2BGR)

        return saturated_image

    def run(self):
        rospy.spin()

def parse_args():
    parser = argparse.ArgumentParser(description="Process image brightness, contrast, and saturation.")
    parser.add_argument('--brightness', type=int, default=50, help='Brightness adjustment value')
    parser.add_argument('--contrast', type=float, default=1.5, help='Contrast adjustment value')
    parser.add_argument('--saturation', type=float, default=1.5, help='Saturation adjustment value')
    args, unknown = parser.parse_known_args()
    return args

if __name__ == '__main__':
    args = parse_args()
    try:
        img_preprocessor = ImagePreProcessor(brightness=args.brightness, contrast=args.contrast, saturation=args.saturation)
        img_preprocessor.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Image PreProcessor node terminated.")
