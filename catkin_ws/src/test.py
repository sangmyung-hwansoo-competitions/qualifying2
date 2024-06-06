import rosbag

# bag 파일 경로 설정
bag_file_path = '/home/ubuntu/catkin_ws/src/integration/bag/usb_cam_recorded_data2.bag'
image_topic = '/usb_cam/image_raw'  # 이미지 메시지가 포함된 토픽

# bag 파일 열기
with rosbag.Bag(bag_file_path, 'r') as bag:
    # 첫 번째와 마지막 메시지의 타임스탬프 초기화
    first_timestamp = None
    last_timestamp = None
    # 이미지 메시지의 개수 초기화
    image_count = 0

    # 지정된 토픽의 메시지 읽기
    for topic, msg, t in bag.read_messages(topics=[image_topic]):
        if first_timestamp is None:
            first_timestamp = t
        last_timestamp = t
        image_count += 1

# 총 시간 (초) 계산
total_time = (last_timestamp - first_timestamp).to_sec()

# FPS 계산
if total_time > 0:
    fps = image_count / total_time
else:
    fps = 0

# FPS 출력
print(f"Total number of image frames: {image_count}")
print(f"Total time: {total_time:.2f} seconds")
print(f"Estimated FPS: {fps:.2f}")
