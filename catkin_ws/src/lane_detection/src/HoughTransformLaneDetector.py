# #!/usr/bin/env python

# # import cv2
# # import numpy as np
# # import yaml

# # class HoughTransformLaneDetector:
# #     def __init__(self, config_path):
# #         with open(config_path, 'r') as file:
# #             config = yaml.safe_load(file)
        
# #         self.mImageWidth = config["IMAGE"]["WIDTH"]
# #         self.mImageHeight = config["IMAGE"]["HEIGHT"]
# #         self.mROIStartHeight = config["IMAGE"]["ROI_START_HEIGHT"]
# #         self.mROIHeight = config["IMAGE"]["ROI_HEIGHT"]
# #         self.mCannyEdgeLowThreshold = config["CANNY"]["LOW_THRESHOLD"]
# #         self.mCannyEdgeHighThreshold = config["CANNY"]["HIGH_THRESHOLD"]
# #         self.mHoughLineSlopeRange = config["HOUGH"]["ABS_SLOPE_RANGE"]
# #         self.mHoughThreshold = config["HOUGH"]["THRESHOLD"]
# #         self.mHoughMinLineLength = config["HOUGH"]["MIN_LINE_LENGTH"]
# #         self.mHoughMaxLineGap = config["HOUGH"]["MAX_LINE_GAP"]
# #         self.mDebugging = config["DEBUG"]
    
# #     def detect_lanes(self, frame):
# #         gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
# #         canny_image = cv2.Canny(gray_image, self.mCannyEdgeLowThreshold, self.mCannyEdgeHighThreshold)
# #         roi = canny_image[self.mROIStartHeight:self.mROIStartHeight + self.mROIHeight, :]
        
# #         lines = cv2.HoughLinesP(roi, 1, np.pi / 180, self.mHoughThreshold, minLineLength=self.mHoughMinLineLength, maxLineGap=self.mHoughMaxLineGap)
# #         left_lines = []
# #         right_lines = []
        
# #         if lines is not None:
# #             for line in lines:
# #                 x1, y1, x2, y2 = line[0]
# #                 slope = (y2 - y1) / (x2 - x1) if (x2 - x1) != 0 else 0
# #                 if abs(slope) < self.mHoughLineSlopeRange:
# #                     continue
# #                 if slope < 0:
# #                     left_lines.append(line)
# #                 else:
# #                     right_lines.append(line)

# #         left_position_x = self.calculate_position(left_lines)
# #         right_position_x = self.calculate_position(right_lines)
        
# #         return left_position_x, right_position_x, lines
    
# #     def calculate_position(self, lines):
# #         if not lines:
# #             return 0

# #         x_sum = 0
# #         y_sum = 0
# #         m_sum = 0
# #         for line in lines:
# #             x1, y1, x2, y2 = line[0]
# #             x_sum += x1 + x2
# #             y_sum += y1 + y2
# #             m_sum += (y2 - y1) / (x2 - x1)

# #         x_avg = x_sum / (2 * len(lines))
# #         y_avg = y_sum / (2 * len(lines))
# #         m = m_sum / len(lines)
# #         b = y_avg - m * x_avg
# #         y = self.mROIHeight / 2
# #         return int((y - b) / m)
    
    
# #     def draw_lanes(self, frame, lines):
# #         if lines is not None:
# #             for line in lines:
# #                 x1, y1, x2, y2 = line[0]
# #                 cv2.line(frame, (x1, y1 + self.mROIStartHeight), (x2, y2 + self.mROIStartHeight), (0, 255, 0), 2)
# #         return frame



# import cv2
# import numpy as np
# import yaml
# import rosbag
# from cv_bridge import CvBridge

# class HoughTransformLaneDetector:
#     def __init__(self, config_path):
#         with open(config_path, 'r') as file:
#             config = yaml.safe_load(file)
        
#         self.mImageWidth = config["IMAGE"]["WIDTH"]
#         self.mImageHeight = config["IMAGE"]["HEIGHT"]
#         self.mROIStartHeight = config["IMAGE"]["ROI_START_HEIGHT"]
#         self.mROIHeight = config["IMAGE"]["ROI_HEIGHT"]
#         self.mCannyEdgeLowThreshold = config["CANNY"]["LOW_THRESHOLD"]
#         self.mCannyEdgeHighThreshold = config["CANNY"]["HIGH_THRESHOLD"]
#         self.mHoughLineSlopeRange = config["HOUGH"]["ABS_SLOPE_RANGE"]
#         self.mHoughThreshold = config["HOUGH"]["THRESHOLD"]
#         self.mHoughMinLineLength = config["HOUGH"]["MIN_LINE_LENGTH"]
#         self.mHoughMaxLineGap = config["HOUGH"]["MAX_LINE_GAP"]
#         self.mDebugging = config["DEBUG"]

#         self.bridge = CvBridge()

#         #원근 변환을 위한 포인트 설정
#         self.src_points = np.float32([
#             [20, self.mROIStartHeight],
#             [self.mImageWidth -20, self.mROIStartHeight],
#             [0, self.mImageHeight],
#             [self.mImageWidth, self.mImageHeight]
#         ])
        
        
#         # self.src_points = np.float32([
#         #     [self.mImageWidth * 0.1, self.mROIStartHeight],  # 좌측 위
#         #     [self.mImageWidth * 0.9, self.mROIStartHeight],  # 우측 위
#         #     [0, self.mImageHeight],  # 좌측 아래
#         #     [self.mImageWidth, self.mImageHeight]  # 우측 아래
#         # ])
#         self.dst_points = np.float32([
#             [0, 0],
#             [self.mImageWidth, 0],
#             [0, self.mImageHeight],
#             [self.mImageWidth, self.mImageHeight]
#         ])
#         self.perspective_transform_matrix = cv2.getPerspectiveTransform(self.src_points, self.dst_points)

#     def detect_lanes(self, frame):
#         birdseye_view = cv2.warpPerspective(frame, self.perspective_transform_matrix, (self.mImageWidth, self.mImageHeight))
#         gray_image = cv2.cvtColor(birdseye_view, cv2.COLOR_BGR2GRAY)
#         canny_image = cv2.Canny(gray_image, self.mCannyEdgeLowThreshold, self.mCannyEdgeHighThreshold)
#         roi = canny_image[self.mROIStartHeight:self.mROIStartHeight + self.mROIHeight, :]
        
#         lines = cv2.HoughLinesP(roi, 1, np.pi / 180, self.mHoughThreshold, minLineLength=self.mHoughMinLineLength, maxLineGap=self.mHoughMaxLineGap)
#         left_lines = []
#         right_lines = []
        
#         if lines is not None:
#             for line in lines:
#                 x1, y1, x2, y2 = line[0]
#                 slope = (y2 - y1) / (x2 - x1) if (x2 - x1) != 0 else 0
#                 if abs(slope) < self.mHoughLineSlopeRange:
#                     continue
#                 if slope < 0:
#                     left_lines.append(line)
#                 else:
#                     right_lines.append(line)

#         left_position_x = self.calculate_position(left_lines)
#         right_position_x = self.calculate_position(right_lines)
        
#         return left_position_x, right_position_x, lines, birdseye_view
    
#     def calculate_position(self, lines):
#         if not lines:
#             return 0

#         x_sum = 0
#         y_sum = 0
#         m_sum = 0
#         for line in lines:
#             x1, y1, x2, y2 = line[0]
#             x_sum += x1 + x2
#             y_sum += y1 + y2
#             m_sum += (y2 - y1) / (x2 - x1)

#         x_avg = x_sum / (2 * len(lines))
#         y_avg = y_sum / (2 * len(lines))
#         m = m_sum / len(lines)
#         b = y_avg - m * x_avg
#         y = self.mROIHeight / 2
#         return int((y - b) / m)
    
#     def draw_lanes(self, frame, lines):
#         if lines is not None:
#             for line in lines:
#                 x1, y1, x2, y2 = line[0]
#                 cv2.line(frame, (x1, y1 + self.mROIStartHeight), (x2, y2 + self.mROIStartHeight), (0, 255, 0), 2)
#         return frame

#     def draw_lanes_birdseye(self, frame, lines):
#         if lines is not None:
#             for line in lines:
#                 x1, y1, x2, y2 = line[0]
#                 cv2.line(frame, (x1, y1+ self.mROIStartHeight), (x2, y2+ self.mROIStartHeight), (0, 255, 0), 2)
#         return frame

# def main():
#     config_path = "config.yaml"
#     bag_path = "/home/ubuntu/catkin_ws/src/integration/bag/usb_cam_recorded_data.bag"
#     topic_name = "/usb_cam/image_raw"

#     lane_detector = HoughTransformLaneDetector(config_path)
#     bag = rosbag.Bag(bag_path, 'r')

#     first_image = None
#     for topic, msg, t in bag.read_messages(topics=[topic_name]):
#         first_image = msg
#         break
    
#     if first_image is None:
#         print("No image found in the bag file.")
#         return
    
#     try:
#         frame = lane_detector.bridge.imgmsg_to_cv2(first_image, "bgr8")
#     except CvBridgeError as e:
#         print(f"CvBridge Error: {e}")
#         return

#     left_pos, right_pos, lines, birdseye_view = lane_detector.detect_lanes(frame)
#     frame_with_lanes = lane_detector.draw_lanes(frame, lines)
#     birdseye_with_lanes = lane_detector.draw_lanes_birdseye(birdseye_view, lines)
#     print("시각화 시작합니다")
    
    
#     cv2.imshow("Frame with Lanes", frame_with_lanes)
#     cv2.imshow("Birdseye View with Lanes", birdseye_with_lanes)
#     while True:
#         key = cv2.waitKey(0)
#         if key == ord('q'):  # 'q' 키가 눌리면 루프 종료
#             break
#     cv2.destroyAllWindows()

# if __name__ == "__main__":
#     main()
