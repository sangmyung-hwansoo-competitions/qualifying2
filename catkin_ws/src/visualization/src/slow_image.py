import rosbag
import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image
import numpy as np

# 원본 bag 파일 경로
input_bag_path = '/home/ubuntu/catkin_ws/src/integration/bag/usb_cam_recorded_data.bag'
# 새로운 bag 파일 경로
output_bag_path = '/home/ubuntu/catkin_ws/src/integration/bag/usb_cam_recorded_data_slow.bag'

bridge = CvBridge()
slow_motion_factor = 2  # 슬로우 모션 배율 (예: 2배 슬로우 모션)

# 새로운 bag 파일을 열기
output_bag = rosbag.Bag(output_bag_path, 'w')

# 원본 bag 파일을 읽기
frames = []
timestamps = []

with rosbag.Bag(input_bag_path, 'r') as input_bag:
    for topic, msg, t in input_bag.read_messages(topics=['/usb_cam/image_raw']):
        # ROS Image 메시지를 OpenCV 이미지로 변환
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frames.append(cv_image)
        timestamps.append(t)

# 첫 번째와 두 번째 프레임 간의 시간 간격 계산
if len(timestamps) > 1:
    time_increment = (timestamps[1] - timestamps[0]).to_sec() / slow_motion_factor
else:
    raise ValueError("The bag file should contain at least two frames.")

# 슬로우 모션 처리를 위해 새로운 타임스탬프 생성
new_timestamps = []
initial_time = timestamps[0]  # 원본 bag 파일의 첫 번째 타임스탬프 사용

for i in range(len(frames)):
    for j in range(slow_motion_factor):
        new_time = initial_time + rospy.Duration.from_sec(i * slow_motion_factor * time_increment + j * time_increment)
        new_timestamps.append(new_time)

# 새로운 메시지를 생성 및 기록
frame_idx = 0
for new_time in new_timestamps:
    frame = frames[frame_idx // slow_motion_factor]
    new_msg = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
    new_msg.header.stamp = new_time
    output_bag.write('/usb_cam/image_raw', new_msg, new_time)
    frame_idx += 1

# 새로운 bag 파일 닫기
output_bag.close()
print("Slow motion bag file has been created successfully.")

# 새로운 bag 파일의 전체 프레임 수 확인
frame_count = 0
with rosbag.Bag(output_bag_path, 'r') as output_bag:
    for topic, msg, t in output_bag.read_messages(topics=['/usb_cam/image_raw']):
        frame_count += 1

print(f"Total number of frames in the new bag file: {frame_count}")
