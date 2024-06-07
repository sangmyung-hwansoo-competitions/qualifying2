#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
import numpy as np
from perception_utils import *
from moving_average_filter import MovingAverageFilter, FilteringMode


class LaneDetector:
    def __init__(self):
        self.node_name = "lane_detector"
        rospy.init_node(self.node_name, anonymous=True)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/image_processor/image_processed", Image, self.callback)
        self.image_pub = rospy.Publisher('/lane_detector/image_detected', Image, queue_size=1)
        self.coord_pub = rospy.Publisher('/lane_detector/coords', Point, queue_size=1)

        self.point = Point()

        self.previous_left_line = None
        self.previous_right_line = None

        self.last_processed_msg = None  # 마지막으로 발행된 이미지 메시지 저장
        self.last_point = Point()  # 마지막으로 발행된 Point 메시지 저장

        self.rx_filter = MovingAverageFilter(25, FilteringMode.WEIGHTED)
        self.lx_filter = MovingAverageFilter(25, FilteringMode.WEIGHTED)

        self.debug = True

    def callback(self, data):
        try:
            canny_img = self.bridge.imgmsg_to_cv2(data, "mono8")
            height, width = canny_img.shape[:2]

            ROI_img = roi(canny_img) # ROI 설정

            line_arr = hough_lines(ROI_img, 1, 1 * np.pi/180, 45, 120, 60) # 허프 변환

            if line_arr is not None:
                temp = np.zeros((canny_img.shape[0], canny_img.shape[1], 3), dtype=np.uint8)

                if line_arr.shape[0] == 1: # 검출된 직선 1개일 때
                    line_arr = np.squeeze(line_arr, axis=0)
                    
                else:
                    line_arr = np.squeeze(line_arr)
   
                # 기울기 구하기
                slope_degree = (np.arctan2(line_arr[:, 1] - line_arr[:, 3], line_arr[:, 0] - line_arr[:, 2]) * 180) / np.pi

                # 수평 기울기 제한
                line_arr = line_arr[np.abs(slope_degree) < 140]
                slope_degree = slope_degree[np.abs(slope_degree) < 140]

                # 수직 기울기 제한
                line_arr = line_arr[np.abs(slope_degree) > 97]
                slope_degree = slope_degree[np.abs(slope_degree) > 97]

                # 필터링된 직선 버리기
                L_lines, R_lines = line_arr[(slope_degree > 0), :], line_arr[(slope_degree < 0), :]
                temp = np.zeros((canny_img.shape[0], canny_img.shape[1], 3), dtype=np.uint8)
                L_lines, R_lines = L_lines[:, None], R_lines[:, None]
                
                # 중앙에서 가장 가까운 직선 2개 중에서 더 긴 직선 선택
                representative_left_line = select_representative_line(L_lines, width / 2)
                representative_right_line = select_representative_line(R_lines, width / 2)

                # 이전 프레임의 대표선과 비교했을때, 하단의 x 좌표가 임계값 이상 차이날 시 이전 프레임의 대표선을 채택
                x_threshold = 150
                if representative_left_line is not None and self.previous_left_line is not None:
                    if abs(representative_left_line[0][0] - self.previous_left_line[0][0]) > x_threshold:
                        representative_left_line = self.previous_left_line

                if representative_right_line is not None and self.previous_right_line is not None:
                    if abs(representative_right_line[0][0] - self.previous_right_line[0][0]) > x_threshold:
                        representative_right_line = self.previous_right_line

                # 검출된 직선이 없는 경우: 이전 프레임의 직선을 대표선으로 채택
                if representative_left_line is None and self.previous_left_line is not None:
                    representative_left_line = self.previous_left_line
                if representative_right_line is None and self.previous_right_line is not None:
                    representative_right_line = self.previous_right_line

                # # 이전 프레임의 대표선과 비교했을 때, 기울기가 임계값 이상상 차이날 시 이전 프레임의 대표선을 채택
                # slope_threshold = 3.5 # 임계값 설정
                # if representative_left_line is not None and self.previous_left_line is not None:
                #     current_slope_left = calculate_slope(representative_left_line.reshape(-1))
                #     previous_slope_left = calculate_slope(self.previous_left_line.reshape(-1))
                #     if abs(current_slope_left - previous_slope_left) > slope_threshold:
                #         representative_left_line = self.previous_left_line

                # if representative_right_line is not None and self.previous_right_line is not None:
                #     current_slope_right = calculate_slope(representative_right_line.reshape(-1))
                #     previous_slope_right = calculate_slope(self.previous_right_line.reshape(-1))
                #     if abs(current_slope_right - previous_slope_right) > slope_threshold:
                #         representative_right_line = self.previous_right_line

                # 직선 그리기
                if representative_left_line is not None:
                    draw_lines(temp, [representative_left_line], color=[0, 0, 255])
                    self.previous_left_line = representative_left_line  # 업데이트
                if representative_right_line is not None:
                    draw_lines(temp, [representative_right_line], color=[0, 255, 0])
                    self.previous_right_line = representative_right_line  # 업데이트  

                y_target = 185
                x_left_at_y = calculate_x_at_y(y_target, representative_left_line)
                x_right_at_y = calculate_x_at_y(y_target, representative_right_line)
                
                self.lx_filter.add_sample(x_left_at_y)
                self.rx_filter.add_sample(x_right_at_y)
                smoothed_lx = self.lx_filter.filtering_result
                smoothed_rx = self.rx_filter.filtering_result
                
                smoothed_x = int((smoothed_lx + smoothed_rx)/2)

                if smoothed_lx is not None and smoothed_rx is not None:
                    cv2.circle(temp, (smoothed_x, y_target), radius=10, color=(0, 255, 0), thickness=-1)
                    cv2.circle(temp, (400, y_target), radius=10, color=(0, 0, 255), thickness=-1)

                processed_image = weighted_img(temp, cv2.cvtColor(canny_img, cv2.COLOR_GRAY2BGR))  # 원본 이미지에 검출된 선 overlap

                processed_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding='bgr8')

                self.last_processed_msg = processed_msg
                self.point.x = smoothed_x
                self.point.y = y_target
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
