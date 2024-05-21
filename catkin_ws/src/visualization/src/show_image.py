#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ImageReceiver:
    def __init__(self):
        self.node_name = "image_receiver"
        rospy.init_node(self.node_name)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        cv2.imshow("Image Window", cv_image)
        cv2.waitKey(1)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        image_receiver = ImageReceiver()
        image_receiver.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
