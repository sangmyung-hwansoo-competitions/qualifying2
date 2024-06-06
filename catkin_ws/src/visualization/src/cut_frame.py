import rosbag

input_bag = '/home/ubuntu/catkin_ws/src/integration/bag/usb_cam_recorded_data_process.bag'
output_bag = '/home/ubuntu/catkin_ws/src/integration/bag/usb_cam_recorded_data_process_cut.bag'
#num_frames_to_skip = 1300  # 건너뛸 프레임 수
num_frames_to_skip = 3000  # 건너뛸 프레임 수

with rosbag.Bag(output_bag, 'w') as outbag:
    frame_count = 0
    for topic, msg, t in rosbag.Bag(input_bag).read_messages():
        frame_count += 1
        if frame_count > num_frames_to_skip:
            outbag.write(topic, msg, t)