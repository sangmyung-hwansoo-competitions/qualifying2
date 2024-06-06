#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import argparse
import numpy as np
import cv2
import rospy
from xycar_msgs.msg import xycar_motor
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
import math
from pid_controller import PIDController

class LaneKeepingControl:
    def __init__(self):
        self.node_name = "lane_keeping_control"
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(30)

        self.start_signal_sub = rospy.Subscriber("/start_signal", Bool, self.start_signal_callback)
        self.coords_sub = rospy.Subscriber("/lane_detector/coords", Point, self.callback)
        self.motor_pub = rospy.Publisher('/xycar_motor', xycar_motor, queue_size=1)

        self.pid = PIDController(0.0012, 0.0000015, 0.0003)
        self.kXycarSteeringAngleLimit = 50

        self.c_coords = None
        self.xycar_msg = xycar_motor()
        self.steer_angle = 0
        self.start_signal = False
        
        self.current_speed = 0.0
        self.min_speed = 0.3
        self.max_speed = 0.55
        
        self.speed_control_threshold = 0.005
        self.deceleration_step = 0.01
        self.acceleration_step = 0.005

    def start_signal_callback(self, msg):
        self.start_signal = msg.data

    def callback(self, data):
        if self.start_signal:
            self.c_coords = data
            if self.c_coords != None:
                self.update_control()

    def update_control(self):
        error_from_mid = self.c_coords.x - 400
        self.steer_angle = self.calculate_steer_angle(error_from_mid)
        # self.steer_angle = 0    # 출발 테스트 용
        self.speed_control(self.steer_angle)

        self.drive(self.steer_angle, self.current_speed)

        rospy.loginfo(f"efm: {error_from_mid}, Steering Angle: {self.steer_angle}, Speed: {self.current_speed}")

    def speed_control(self, steering_angle):
        if abs(steering_angle) > self.speed_control_threshold:
            self.current_speed -= self.deceleration_step
            self.current_speed = max(self.current_speed, self.min_speed)
        else:
            self.current_speed += self.acceleration_step
            self.current_speed = min(self.current_speed, self.max_speed)


    def calculate_steer_angle(self, error_from_mid):
        control_output = self.pid.get_control_output(error_from_mid)
        steer_angle = max(-self.kXycarSteeringAngleLimit, min(control_output, self.kXycarSteeringAngleLimit))
        return steer_angle

    def drive(self, angle, speed):
        self.xycar_msg.header.stamp = rospy.Time.now()
        self.xycar_msg.angle = angle
        self.xycar_msg.speed = speed
        self.motor_pub.publish(self.xycar_msg)

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()
            self.rate.sleep()


if __name__ == '__main__':
    try:
        control = LaneKeepingControl()
        control.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Lane Keeping Control node terminated.")
