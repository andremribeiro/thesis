#!/usr/bin/env python

import rosbag
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry

bag_path = '/home/andre/thesis/one_uav.bag'

data = {
    'time_odom': [],
    'x1': [], 'y1': [], 'z1': [],
    'time_coverage1': [], 'occupied1': [], 'free1': [], 'missing1': [], 'mapped1': [],
}

total_distance = 0

with rosbag.Bag(bag_path, 'r') as bag:
    first_timestamp_coverage = None
    prev_position = None
    for topic, msg, t in bag.read_messages(topics=['/uav1/estimation_manager/odom_main', '/mappingCoverage1']):
        try:
            if topic == '/uav1/estimation_manager/odom_main':
                data['time_odom'].append(t.to_sec())
                data['x1'].append(msg.pose.pose.position.x)
                data['y1'].append(msg.pose.pose.position.y)
                data['z1'].append(msg.pose.pose.position.z)
                # Calculate distance
                curr_position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
                if prev_position is not None:
                    total_distance += np.linalg.norm(curr_position - prev_position)
                prev_position = curr_position
            elif topic == '/mappingCoverage1':
                if first_timestamp_coverage is None:
                    first_timestamp_coverage = msg.data[0]
                data['time_coverage1'].append(msg.data[0] - first_timestamp_coverage)
                data['occupied1'].append(msg.data[1] * 100)
                data['free1'].append(msg.data[2] * 100)
                data['mapped1'].append((msg.data[1] + msg.data[2]) * 100)
                data['missing1'].append(msg.data[3] * 100)
        except Exception as e:
            print("Failed to append data: ", e)

# Print total distance
print("Total distance travelled by the UAV: ", total_distance, "meters")

print("Total time travelled by the UAV: ", data['time_coverage1'][-1], "seconds")

print("Total coverage by the UAV: ", data['mapped1'][-1], "percent")

# Create a figure and axes
fig1 = plt.figure()
ax1 = fig1.add_subplot(111, projection='3d')

# Plot odometry data
ax1.scatter(data['x1'], data['y1'], data['z1'], s=1, color='blue')

# Set labels and title
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_zlabel('Z')
ax1.set_title('Odometry Data')

# Set the limits of the 3D plot
ax1.set_xlim([-30, 30])
ax1.set_ylim([-30, 30])
ax1.set_zlim([0, 10])

# Show the plot
plt.show(block=False)

# Create a figure and axes for cell counts
fig2, ax2 = plt.subplots()

# Plot cell counts
ax2.plot(data['time_coverage1'], data['mapped1'], label='Mapped Cells', color='blue', linestyle='-')
ax2.plot(data['time_coverage1'], data['free1'], label='Free Cells', color='cyan', linestyle='--')
ax2.plot(data['time_coverage1'], data['occupied1'], label='Occupied Cells', color='red', linestyle='--')
ax2.plot(data['time_coverage1'], data['missing1'], label='Unknown Cells', color='orange', linestyle='-')

# Set labels and title for cell counts
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Cell Count (%)')
ax2.set_title('Mapping Coverage')

# Add a legend for cell counts
ax2.legend()

# Show the cell counts plot
plt.show(block=True)
