#!/usr/bin/env python

import rosbag
import octomap
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry

# Path to your bag file
bag_path = '/home/andre/thesis/2024-02-07-10-30-08.bag'

# Load the OctoMap
tree = octomap.OcTree("/home/andre/mapping_ws/mapfile.bt".encode())

# Initialize lists to hold data
tree_x_data = []
tree_y_data = []
tree_z_data = []

# Iterate over all occupied cells
for node in tree.begin_tree():
    if tree.isNodeOccupied(node):
        # Get the coordinates of the cell
        x, y, z = tree.keyToCoord(node.getKey())
        tree_x_data.append(x)
        tree_y_data.append(y)
        tree_z_data.append(z)

# Create a figure and axes
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot data
ax.scatter(tree_x_data, tree_y_data, tree_z_data, s=1)

# Set labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('OctoMap Data')

# Set the limits of the 3D plot
ax.set_xlim([-30, 30])
ax.set_ylim([-30, 30])
ax.set_zlim([0, 12.5])

# Show the plot
plt.show(block=False)

# Initialize lists to hold data
time_data_odom = []
x_data = []
y_data = []
z_data = []
time_data_coverage = []
occupiedCells_data = []
freeCells_data = []
missingCells_data = []
mappedCells_data = []

# Open the bag file
with rosbag.Bag(bag_path, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=['/uav1/estimation_manager/odom_main', '/mappingCoverage']):
        try:
            if topic == '/uav1/estimation_manager/odom_main':
                # Append data to lists
                time_data_odom.append(t.to_sec())
                x_data.append(msg.pose.pose.position.x)
                y_data.append(msg.pose.pose.position.y)
                z_data.append(msg.pose.pose.position.z)
            elif topic == '/mappingCoverage':
                # Append data to lists
                time_data_coverage.append(msg.data[0])
                occupiedCells_data.append(msg.data[1])
                freeCells_data.append(msg.data[2])
                mappedCells_data.append(msg.data[1]+msg.data[2])
                missingCells_data.append(msg.data[3])
        except Exception as e:
            print("Failed to append data: ", e)

# Create a figure and axes for odometry data
fig1 = plt.figure()
ax1 = fig1.add_subplot(111, projection='3d')

# Plot odometry data
ax1.plot(x_data, y_data, z_data, label='Odometry Data')

# Set labels and title for odometry data
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_zlabel('Z')
ax1.set_title('Odometry Data')

# Set the limits of the 3D plot
ax1.set_xlim([-30, 30])
ax1.set_ylim([-30, 30])
ax1.set_zlim([0, 12.5])

# Add a legend for odometry data
ax1.legend()

# Show the odometry plot
plt.show(block=False)

# Create a figure and axes for cell counts
fig2, ax2 = plt.subplots()

# Plot cell counts
ax2.plot(time_data_coverage, occupiedCells_data, label='Occupied Cells')
ax2.plot(time_data_coverage, freeCells_data, label='Free Cells')
ax2.plot(time_data_coverage, missingCells_data, label='Missing Cells')
ax2.plot(time_data_coverage, mappedCells_data, label='Mapped Cells')

# Set labels and title for cell counts
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Cell Count')
ax2.set_title('Cell Counts vs Time')

# Add a legend for cell counts
ax2.legend()

# Show the cell counts plot
plt.show(block=False)
