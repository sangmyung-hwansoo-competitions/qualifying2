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

# Callback function for the image subscriber
def image_callback(msg):
    global image_pub
    try:
        # Convert ROS Image message to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        ##############################################################
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
            line_arr = np.squeeze(line_arr)
            
            if line_arr.ndim == 1: # 1차원 배열을 2차원 배열로 변경 (개수 1개일 때 예외처리)
                line_arr = line_arr[np.newaxis, :]
            
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




