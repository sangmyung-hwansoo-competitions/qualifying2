#!/usr/bin/env python3

from lane_detection_utils import *
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image
import numpy as np

# Initialize the CvBridge
bridge = CvBridge()

# ROS Publisher for the processed image
image_pub = None

coord_pub = None

# 직선 데이터 저장용 변수
previous_left_line = None
previous_right_line = None


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


# Callback function for the image subscriber
def image_callback(msg):
    global image_pub
    # 직선 데이터 저장용 변수
    global previous_left_line
    global previous_right_line
    try:
        # Convert ROS Image message to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        processed_image = bev_transform(cv_image) # Bird's Eye View 변환 적용
        
        height, width = cv_image.shape[:2] # 이미지 높이, 너비
        
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
            temp = np.zeros((cv_image.shape[0], cv_image.shape[1], 3), dtype=np.uint8)
            L_lines, R_lines = L_lines[:, None], R_lines[:, None]

            # 검출된 직선이 1개인 경우: 한 프레임 이전 직선 추가해서 검출 직선 2개로 고정
            if len(L_lines) == 1 and previous_left_line is not None:
                L_lines = np.append(L_lines, [previous_left_line], axis=0)
            if len(R_lines) == 1 and previous_right_line is not None:
                R_lines = np.append(R_lines, [previous_right_line], axis=0)
            
            # 중앙에서 가장 가까운 직선 2개 중에서 더 긴 직선 대표선으로 선택
            representative_left_line = select_representative_line(L_lines, width / 2)
            representative_right_line = select_representative_line(R_lines, width / 2)
            
            # 이전 프레임의 대표선과 비교했을때, 하단의 x 좌표가 임계값 이상 차이날 시 이전 프레임의 대표선을 채택
            x_threshold = 80
            if representative_left_line is not None and previous_left_line is not None:
                if abs(representative_left_line[0][0] - previous_left_line[0][0]) > x_threshold:
                    representative_left_line = previous_left_line

            if representative_right_line is not None and previous_right_line is not None:
                if abs(representative_right_line[0][0] - previous_right_line[0][0]) > x_threshold:
                    representative_right_line = previous_right_line
                    
                    
            # 이전 프레임의 대표선과 비교했을 때, 기울기가 임계값 이하 차이날 시 이전 프레임의 대표선을 채택
            slope_threshold = 0.1  # 임계값 설정
            if representative_left_line is not None and previous_left_line is not None:
                current_slope_left = calculate_slope(representative_left_line.reshape(-1))
                previous_slope_left = calculate_slope(previous_left_line.reshape(-1))
                if abs(current_slope_left - previous_slope_left) < slope_threshold:
                    representative_left_line = previous_left_line

            if representative_right_line is not None and previous_right_line is not None:
                current_slope_right = calculate_slope(representative_right_line.reshape(-1))
                previous_slope_right = calculate_slope(previous_right_line.reshape(-1))
                if abs(current_slope_right - previous_slope_right) < slope_threshold:
                    representative_right_line = previous_right_line
     
             # 검출된 직선이 없는 경우: 이전 프레임의 직선을 대표선으로 채택
            if representative_left_line is None and previous_left_line is not None:
                representative_left_line = previous_left_line
            if representative_right_line is None and previous_right_line is not None:
                representative_right_line = previous_right_line
            
            # 직선 그리기
            if representative_left_line is not None:
                draw_lines(temp, [representative_left_line], color=[0, 0, 255])
                previous_left_line = representative_left_line  # 업데이트
            if representative_right_line is not None:
                draw_lines(temp, [representative_right_line], color=[0, 255, 0])
                previous_right_line = representative_right_line  # 업데이트      
            
            
            # 중앙점
            x_filter = MovingAverageFilter(window_size=40)
            y_filter = MovingAverageFilter(window_size=50)
            
            # f_data = np.append(representative_left_line, representative_right_line)
            # # ex: [ 18 350 186  60 480 102 519 328]
            # f_data = [float(inp) for inp in f_data]
            # # ex: [18.0, 350.0, 186.0, 60.0, 480.0, 102.0, 519.0, 328.0]
            
            # x1l, y1l, x2l, y2l, x1r, y1r, x2r, y2r = f_data
            # center_coord = ((x1l + x2l + x1r + x2r) / 4, (y1l + y2l + y1r + y2r) / 4)
            
            
            y=300
            slope_left, intercept_left = calculate_line_equation(representative_left_line.reshape(-1))
            slope_right, intercept_right = calculate_line_equation(representative_right_line.reshape(-1))
            
            x_left = calculate_x_at_y(slope_left, intercept_left, y)
            x_right = calculate_x_at_y(slope_right, intercept_right, y)
            
            center_coord = ((x_left + x_right) / 2, y)
            
            
            
            # Smooth the coordinates using the filters
            smoothed_x = int(x_filter.add_data(center_coord[0]))
            smoothed_y = int(y_filter.add_data(center_coord[1]))
            
            if smoothed_x is not None and smoothed_y is not None:
                cv2.circle(temp, (smoothed_x, smoothed_y), radius=10, color=(0, 255, 0), thickness=-1)  
            
            
            processed_image = weighted_img(temp, cv2.cvtColor(canny_img, cv2.COLOR_GRAY2BGR))  # 원본 이미지에 검출된 선 overlap
        else:
            processed_image = cv_image  # 직선이 검출되지 않으면 원본 이미지 사용
        
        # Convert processed image back to ROS Image message
        processed_msg = bridge.cv2_to_imgmsg(processed_image, encoding='bgr8')
        
        # Publish the processed image
        image_pub.publish(processed_msg)
    
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

def main():
    global image_pub
    rospy.init_node('lane_detection_node', anonymous=True)

    # Subscribe to the raw image topic
    rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)
    
    # Create a publisher for the processed image
    image_pub = rospy.Publisher('/lane_detection/image_processed', Image, queue_size=1)
    
    rospy.spin()

if __name__ == '__main__':
    main()




