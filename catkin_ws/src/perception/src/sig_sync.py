#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image, Imu
from message_filters import ApproximateTimeSynchronizer, Subscriber

def callback(image, imu, image_pub, imu_pub):
    # Process synchronized image and IMU data
    # rospy.loginfo("Synchronized Image and IMU data received")

    # Publish the synchronized messages
    image_pub.publish(image)
    imu_pub.publish(imu)

def listener():
    rospy.init_node('sync_listener', anonymous=True)

    image_sub = Subscriber('/usb_cam/image_raw', Image)
    imu_sub = Subscriber('/imu', Imu)

    image_pub = rospy.Publisher('/sync/image', Image, queue_size=10)
    imu_pub = rospy.Publisher('/sync/imu', Imu, queue_size=10)

    ats = ApproximateTimeSynchronizer([image_sub, imu_sub], queue_size=10, slop=0.1)
    ats.registerCallback(callback, image_pub, imu_pub)

    rospy.spin()

if __name__ == '__main__':
    listener()