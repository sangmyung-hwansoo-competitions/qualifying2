#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from lane_detection_utils import *
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
import numpy as np


class MovingAverageFilter:
    def __init__(self, window_size):
        self.window_size = window_size
        self.data = []

    def add_data(self, value):
        if len(self.data) >= self.window_size:
            self.data.pop(0)
        self.data.append(value)
        return self.get_average()

    def get_average(self):
        if not self.data:
            return None
        return sum(self.data) / len(self.data)


class LaneDetector:
    def __init__(self):
        self.node_name = "lane_detection"
        rospy.init_node(self.node_name, anonymous=True)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)
        self.image_pub = rospy.Publisher('/lane_detection/image_processed', Image, queue_size=1)
        self.coord_pub = rospy.Publisher('/lane_detection/coords', Point, queue_size=10)

        self.point = Point()
        
        self.last_processed_msg = None  # 마지막으로 발행된 이미지 메시지 저장
        self.last_point = Point()  # 마지막으로 발행된 Point 메시지 저장

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            height, width = cv_image.shape[:2]

            processed_image = bev_transform(cv_image)

            processed_image = adjust_brightness_contrast(processed_image, brightness=-30, contrast=30)
            processed_image = apply_clahe(processed_image)
            processed_image = adjust_gamma(processed_image, gamma=1.2)

            gray_img = grayscale(processed_image) # 흑백이미지로 변환

            blur_img = gaussian_blur(gray_img, 5) # Blur 효과
                    
            canny_img = canny(blur_img, 30, 210) # Canny edge 알고리즘
            
            ROI_img = roi(canny_img) # ROI 설정

            line_arr = hough_lines(ROI_img, 1, 1 * np.pi/180, 20, 150, 15) # 허프 변환

            if line_arr is not None:
                temp = np.zeros((cv_image.shape[0], cv_image.shape[1], 3), dtype=np.uint8)
                line_arr = np.squeeze(line_arr)
                
                # 기울기 구하기
                slope_degree = (np.arctan2(line_arr[:, 1] - line_arr[:, 3], line_arr[:, 0] - line_arr[:, 2]) * 180) / np.pi

                # 수평 기울기 제한
                line_arr = line_arr[np.abs(slope_degree) < 170]
                slope_degree = slope_degree[np.abs(slope_degree) < 170]
                # 수직 기울기 제한
                line_arr = line_arr[np.abs(slope_degree) > 95]
                slope_degree = slope_degree[np.abs(slope_degree) > 95]
                # 필터링된 직선 버리기
                L_lines, R_lines = line_arr[(slope_degree > 0), :], line_arr[(slope_degree < 0), :]
                temp = np.zeros((cv_image.shape[0], cv_image.shape[1], 3), dtype=np.uint8)
                L_lines, R_lines = L_lines[:, None], R_lines[:, None]
                
                # 중앙에서 가장 가까운 직선 2개 중에서 더 긴 직선 선택
                representative_left_line = select_representative_line(L_lines, width / 2)
                representative_right_line = select_representative_line(R_lines, width / 2)

                # 직선 그리기
                if representative_left_line is not None:
                    draw_lines(temp, [representative_left_line], color=[0, 0, 255])
                
                if representative_right_line is not None:
                    draw_lines(temp, [representative_right_line], color=[0, 255, 0])
                
                # 중앙점
                x_filter = MovingAverageFilter(window_size=40)
                y_filter = MovingAverageFilter(window_size=50)
                
                f_data = np.append(representative_left_line, representative_right_line)
                f_data = [float(inp) for inp in f_data]
                
                x1l, y1l, x2l, y2l, x1r, y1r, x2r, y2r = f_data
                center_coord = ((x1l + x2l + x1r + x2r) / 4, (y1l + y2l + y1r + y2r) / 4)
                
                # Smooth the coordinates using the filters
                smoothed_x = int(x_filter.add_data(center_coord[0]))
                smoothed_y = int(y_filter.add_data(center_coord[1]))
                
                if smoothed_x is not None and smoothed_y is not None:
                    cv2.circle(temp, (smoothed_x, smoothed_y), radius=10, color=(0, 255, 0), thickness=-1)
                    cv2.circle(temp, (400, 300), radius=10, color=(0, 0, 255), thickness=-1)
                
                processed_image = weighted_img(temp, cv2.cvtColor(canny_img, cv2.COLOR_GRAY2BGR))  # 원본 이미지에 검출된 선 overlap

                # Convert processed image back to ROS Image message
                processed_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding='bgr8')

                self.last_processed_msg = processed_msg
                self.point.x = smoothed_x
                self.point.y = smoothed_y
                self.last_point = self.point
            else:
                # 직선이 검출되지 않으면 마지막으로 성공적으로 처리된 이미지 사용
                if self.last_processed_msg is not None:
                    self.image_pub.publish(self.last_processed_msg)
                if self.last_point is not None:
                    self.coord_pub.publish(self.last_point)
                return

            self.image_pub.publish(processed_msg)
            self.coord_pub.publish(self.point)

        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")

            # 마지막으로 성공적으로 발행된 메시지 다시 발행
            if self.last_processed_msg is not None:
                self.image_pub.publish(self.last_processed_msg)
            if self.last_point is not None:
                self.coord_pub.publish(self.last_point)
            return

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        lane_detector = LaneDetector()
        lane_detector.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("lane detector node terminated.")
