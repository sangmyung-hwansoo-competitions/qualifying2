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

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0.0
        self.integral = 0.0
        self.derivative = 0.0

    def get_control_output(self, error_from_mid):
        self.integral += error_from_mid
        self.derivative = error_from_mid - self.prev_error
        self.prev_error = error_from_mid

        if abs(error_from_mid) > 100:
            return 0.3 * error_from_mid + self.Ki * self.integral + self.Kd * self.derivative

        if abs(error_from_mid) > 120:
            return 0.6 * error_from_mid + self.Ki * self.integral + self.Kd * self.derivative

        return self.Kp * error_from_mid + self.Ki * self.integral + self.Kd * self.derivative

class LaneKeepingControl:
    def __init__(self):
        self.node_name = "lane_keeping_control"
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)

        self.start_signal_sub = rospy.Subscriber("/start_signal", Bool, self.start_signal_callback)
        self.coords_sub = rospy.Subscriber("/lane_detection/coords", Point, self.callback)
        self.motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

        self.pid = PIDController(0.05, 0.001, 0.01)
        self.kXycarSteeringAngleLimit = 50

        self.c_coords = None
        self.xycar_msg = xycar_motor()
        self.steer_angle = 0
        self.start_signal = False
        
        self.current_speed = 0.0
        self.min_speed = 3.0
        self.max_speed = 5.0
        
        self.speed_control_threshold = 0.15
        self.deceleration_step = 0.15
        self.acceleration_step = 0.02

    def start_signal_callback(self, msg):
        self.start_signal = msg

    def callback(self, data):
        if self.start_signal:
            self.c_coords = data
            print(self.c_coords)
            if self.c_coords != None:
                control_output = self.pid.get_control_output(self.c_coords.x - 400)
                self.steer_angle = max(-self.kXycarSteeringAngleLimit, min(control_output, self.kXycarSteeringAngleLimit))
                print(self.steer_angle)
                # relocated_coord = (self.c_coords.x - 400, 600 - self.c_coords.y)
                # self.steer_angle = math.atan2(*relocated_coord) * 0.75
                speed = self.speed_control(self.steer_angle)
                # self.drive(self.steer_angle, speed)

    def speed_control(self, steering_angle):
        if abs(steering_angle) > self.speed_control_threshold:
            self.current_speed -= self.deceleration_step
            self.current_speed = max(self.current_speed, self.min_speed)
        else:
            self.current_speed += self.acceleration_step
            self.current_speed = min(self.current_speed, self.max_speed)
        
        return self.current_speed

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
