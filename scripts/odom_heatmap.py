import rosbag
import numpy as np

import matplotlib.pyplot as plt

# List of rosbag files
rosbag_files = ['/home/andre/thesis/bags/1_uav/test_15_1.bag', 
                '/home/andre/thesis/bags/1_uav/test_15_2.bag', 
                '/home/andre/thesis/bags/1_uav/test_15_3.bag', 
                '/home/andre/thesis/bags/1_uav/test_15_4.bag', 
                '/home/andre/thesis/bags/1_uav/test_15_5.bag']

# Initialize empty lists for x and y positions
x_positions = []
y_positions = []
z_positions = []

# Iterate over each rosbag file
for rosbag_file in rosbag_files:
    # Open the rosbag file
    bag = rosbag.Bag(rosbag_file)

    # Iterate over each message in the rosbag
    for topic, msg, t in bag.read_messages(topics=['/uav1/estimation_manager/odom_main']):
        # Extract the x and y positions from the odom message
        x_positions.append(msg.pose.pose.position.x)
        y_positions.append(msg.pose.pose.position.y)
        z_positions.append(msg.pose.pose.position.z)

    # Close the rosbag file
    bag.close()

# Calculate path lengths
path_lengths = []
for i in range(len(x_positions) - 1):
    dx = x_positions[i+1] - x_positions[i]
    dy = y_positions[i+1] - y_positions[i]
    dz = z_positions[i+1] - z_positions[i]
    path_length = np.sqrt(dx**2 + dy**2 + dz**2)
    path_lengths.append(path_length)

# Calculate average path length
average_path_length = np.mean(path_lengths)

# Calculate standard deviation of path length
std_dev_path_length = np.std(path_lengths)

# Calculate minimum path length
min_path_length = np.min(path_lengths)

# Calculate maximum path length
max_path_length = np.max(path_lengths)

# Print the results
print("Average Path Length:", average_path_length)
print("Standard Deviation of Path Length:", std_dev_path_length)
print("Minimum Path Length:", min_path_length)
print("Maximum Path Length:", max_path_length)