#!/usr/bin/env python

import rosbag
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry

bag_path = '/home/andre/thesis/sims/2_uav/test4.bag'

data = {
    'time_odom': [],
    'x1': [], 'y1': [], 'z1': [],
    'x2': [], 'y2': [], 'z2': [],
    'time_coverage1': [], 'occupied1': [], 'free1': [], 'missing1': [], 'mapped1': [],
    'time_coverage2': [], 'occupied2': [], 'free2': [], 'missing2': [], 'mapped2': [],
    'uavs_in_range1': [], 'uavs_in_range2': [],
}

total_distance1 = 0
total_distance2 = 0

with rosbag.Bag(bag_path, 'r') as bag:
    first_timestamp_coverage = None
    prev_position1 = None
    prev_position2 = None
    for topic, msg, t in bag.read_messages(topics=['/uav1/estimation_manager/odom_main' ,'/uav2/estimation_manager/odom_main', '/uav1/coverage', '/uav2/coverage', '/uav1/uavs_in_range', '/uav2/uavs_in_range']):
        try:
            if topic == '/uav1/estimation_manager/odom_main':
                data['time_odom'].append(t.to_sec())
                data['x1'].append(msg.pose.pose.position.x)
                data['y1'].append(msg.pose.pose.position.y)
                data['z1'].append(msg.pose.pose.position.z)
                # Calculate distance
                curr_position1 = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
                if prev_position1 is not None and data['time_odom'][-1] - data['time_odom'][0] < 139.5:
                    total_distance1 += np.linalg.norm(curr_position1 - prev_position1)
                prev_position1 = curr_position1
            elif topic == '/uav2/estimation_manager/odom_main':
                data['time_odom'].append(t.to_sec())
                data['x2'].append(msg.pose.pose.position.x)
                data['y2'].append(msg.pose.pose.position.y)
                data['z2'].append(msg.pose.pose.position.z)
                # Calculate distance
                curr_position2 = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
                if prev_position2 is not None and data['time_odom'][-1] - data['time_odom'][0] < 139.5:
                    total_distance2 += np.linalg.norm(curr_position2 - prev_position2)
                prev_position2 = curr_position2
            elif topic == '/uav1/coverage':
                if first_timestamp_coverage is None:
                    first_timestamp_coverage = msg.data[0]
                data['time_coverage1'].append(msg.data[0] - first_timestamp_coverage)
                data['occupied1'].append(100 * msg.data[1] / msg.data[4])
                data['free1'].append(100 * msg.data[2] / msg.data[4])
                data['mapped1'].append(100 * (msg.data[1] + msg.data[2]) / msg.data[4])
                data['missing1'].append(100 * msg.data[3] / msg.data[4])
            elif topic == '/uav2/coverage':
                if first_timestamp_coverage is None:
                    first_timestamp_coverage = msg.data[0]
                data['time_coverage2'].append(msg.data[0] - first_timestamp_coverage)
                data['occupied2'].append(100 * msg.data[1] / msg.data[4])
                data['free2'].append(100 * msg.data[2] / msg.data[4])
                data['mapped2'].append(100 * (msg.data[1] + msg.data[2]) / msg.data[4])
                data['missing2'].append(100 * msg.data[3] / msg.data[4])
            elif topic == '/uav1/uavs_in_range':
                data['uavs_in_range1'].extend(msg.data)
            elif topic == '/uav2/uavs_in_range':
                data['uavs_in_range2'].extend(msg.data)
        except Exception as e:
            print("Failed to append data: ", e)

# Print total distance
print("Total distance travelled by UAV1: ", total_distance1, "meters")
print("Total distance travelled by UAV2: ", total_distance2, "meters")

print("Total time travelled by UAV1: ", data['time_coverage1'][-1], "seconds")
print("Total time travelled by UAV2: ", data['time_coverage2'][-1], "seconds")

print("Total coverage by UAV1: ", data['mapped1'][-1], "percent")
print("Total coverage by UAV2: ", data['mapped2'][-1], "percent")

# Create a figure and axes
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Determine the minimum length of the two arrays
min_length = min(len(data['x1']), len(data['x2']))

# Trim the longer array to match the size of the shorter one
data['x1'] = data['x1'][:min_length]
data['y1'] = data['y1'][:min_length]
data['z1'] = data['z1'][:min_length]
data['x2'] = data['x2'][:min_length]
data['y2'] = data['y2'][:min_length]
data['z2'] = data['z2'][:min_length]

# Plot odometry data
ax.scatter(data['x1'], data['y1'], data['z1'], s=1)
ax.scatter(data['x2'], data['y2'], data['z2'], s=1)

# Set labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Odometry Data')

# Set the limits of the 3D plot
ax.set_xlim([-30, 30])
ax.set_ylim([-30, 30])
ax.set_zlim([0, 10])

# Show the plot
plt.show(block=False)

# Create a figure and axes for cell counts
fig2, ax2 = plt.subplots()

# Plot cell counts
ax2.plot(data['time_coverage1'], data['mapped1'], label='UAV1 Mapped Cells', linestyle='solid')
ax2.plot(data['time_coverage2'], data['mapped2'], label='UAV2 Mapped Cells', linestyle='solid')

# Set labels and title for cell counts
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Cell Count (%)')
ax2.set_title('Mapping Coverage')

# Add a legend for cell counts
ax2.legend()

# Show the cell counts plot
plt.show(block=True)
