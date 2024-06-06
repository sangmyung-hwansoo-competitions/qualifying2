import rosbag
import csv
import os

# Define the bag file path and the output CSV file path
bag_file_path = '/home/ubuntu/catkin_ws/src/integration/bag/imu_recorded_data3.bag'
imu_csv_file_path = '/home/ubuntu/catkin_ws/src/integration/bag/imu_data3.csv'

# Define the topic you want to extract
imu_topic = '/imu'

# Open the bag file
with rosbag.Bag(bag_file_path, 'r') as bag:
    with open(imu_csv_file_path, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        
        # Write the header
        csv_writer.writerow([
            '#timestamp [ns]', 'w_RS_S_x [rad s^-1]', 'w_RS_S_y [rad s^-1]', 
            'w_RS_S_z [rad s^-1]', 'a_RS_S_x [m s^-2]', 'a_RS_S_y [m s^-2]', 
            'a_RS_S_z [m s^-2]'
        ])
        
        # Read messages from the bag file
        for topic, msg, t in bag.read_messages(topics=[imu_topic]):
            timestamp = t.to_nsec()
            w_RS_S_x = msg.angular_velocity.x
            w_RS_S_y = msg.angular_velocity.y
            w_RS_S_z = msg.angular_velocity.z
            a_RS_S_x = msg.linear_acceleration.x
            a_RS_S_y = msg.linear_acceleration.y
            a_RS_S_z = msg.linear_acceleration.z
            
            # Write the row to the CSV file
            csv_writer.writerow([
                timestamp, w_RS_S_x, w_RS_S_y, w_RS_S_z, a_RS_S_x, a_RS_S_y, a_RS_S_z
            ])

print(f"IMU data saved to {imu_csv_file_path}")

