# #!/usr/bin/env python3

# import sys
# import os
# sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))
# sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'devel', 'lib', 'python3', 'dist-packages'))

# import rospy
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError
# import cv2
# import numpy as np
# from visualization_msgs.msg import Marker
# from nav_msgs.msg import Path
# from geometry_msgs.msg import PoseStamped, Point
# import yaml
# from HoughTransformLaneDetector import HoughTransformLaneDetector

# class LaneDetectionNode:
#     def __init__(self):
#         self.bridge = CvBridge()
#         self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
#         self.lane_marker_pub = rospy.Publisher("lane_markers", Marker, queue_size=1)
#         self.path_pub = rospy.Publisher("lane_path", Path, queue_size=1)
#         self.image_pub = rospy.Publisher("/processed_image", Image, queue_size=1)  # Initialize image_pub
        
        
#         # Load configuration
#         self.lane_detector = HoughTransformLaneDetector("config.yaml")

#     def image_callback(self, msg):
#         try:
#             frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#         except CvBridgeError as e:
#             rospy.logerr("CvBridge Error: {0}".format(e))
#             return

#         left_position_x, right_position_x, lines = self.lane_detector.detect_lanes(frame)
#         self.publish_lanes(lines)
#         self.publish_path([(left_position_x, self.lane_detector.mImageHeight), (right_position_x, self.lane_detector.mImageHeight)])
        
#         # Draw lanes and path on image
#         image_with_lanes = self.draw_lanes_and_path(frame, lines, [(left_position_x, self.lane_detector.mROIStartHeight + self.lane_detector.mROIHeight),
#                                                                   (right_position_x, self.lane_detector.mROIStartHeight + self.lane_detector.mROIHeight)])
        
#         # Publish the image with lanes
#         try:
#             self.image_pub.publish(self.bridge.cv2_to_imgmsg(image_with_lanes, "bgr8"))
#         except CvBridgeError as e:
#             rospy.logerr("CvBridge Error: {0}".format(e))

#     def draw_lanes_and_path(self, image, lines, path_points):
#         if lines is not None:
#             for line in lines:
#                 x1, y1, x2, y2 = line[0]
#                 cv2.line(image, (x1, y1 + self.lane_detector.mROIStartHeight), (x2, y2 + self.lane_detector.mROIStartHeight), (0, 255, 0), 2)

#         for pt in path_points:
#             cv2.circle(image, (int(pt[0]), int(pt[1])), 5, (0, 0, 255), -1)

#         return image





#     def publish_lanes(self, lines):
#         line_list = Marker()
#         line_list.header.frame_id = "base_link"  # Ensure frame_id is set
#         line_list.header.stamp = rospy.Time.now()
#         line_list.ns = "lane_lines"
#         line_list.action = Marker.ADD
#         line_list.pose.orientation.w = 1.0
#         line_list.id = 0
#         line_list.type = Marker.LINE_LIST

#         line_list.scale.x = 0.1
#         line_list.color.r = 1.0
#         line_list.color.g = 1.0
#         line_list.color.b = 1.0
#         line_list.color.a = 1.0

#         if lines is not None:
#             for line in lines:
#                 x1, y1, x2, y2 = line[0]
#                 p1 = Point(x=x1, y=y1 + self.lane_detector.mROIStartHeight, z=0)
#                 p2 = Point(x=x2, y=y2 + self.lane_detector.mROIStartHeight, z=0)
#                 line_list.points.append(p1)
#                 line_list.points.append(p2)

#         self.lane_marker_pub.publish(line_list)

#     def publish_path(self, path_points):
#         path = Path()
#         path.header.frame_id = "base_link"  # Ensure frame_id is set
#         path.header.stamp = rospy.Time.now()

#         for pt in path_points:
#             pose = PoseStamped()
#             pose.header.frame_id = "base_link"  # Ensure frame_id is set for each pose
#             pose.pose.position.x = float(pt[0])
#             pose.pose.position.y = float(pt[1])
#             pose.pose.position.z = 0
#             pose.pose.orientation.w = 1.0
#             path.poses.append(pose)

#         self.path_pub.publish(path)

# if __name__ == '__main__':
#     rospy.init_node('lane_detection_node', anonymous=True)
#     node = LaneDetectionNode()
#     try:
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass

#!/usr/bin/env python3

from lane_detection_utils import *
import cv2
from cv_bridge import CvBridge
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






