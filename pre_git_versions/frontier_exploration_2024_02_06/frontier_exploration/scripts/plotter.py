#!/usr/bin/env python

import rosbag
import matplotlib.pyplot as plt
from std_msgs.msg import Float64MultiArray

# Path to your bag file
bag_path = '/home/andre/thesis/2024-02-06-14-12-34.bag'

# Initialize lists to hold data
time_data = []
occupiedCells_data = []
freeCells_data = []
totalCells_data = []
missingCells_data = []

# Open the bag file
with rosbag.Bag(bag_path, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=['/mappingCoverage']):
        try:
            # Append data to lists
            time_data.append(msg.data[4])
            occupiedCells_data.append(msg.data[0])
            freeCells_data.append(msg.data[1])
            totalCells_data.append(msg.data[0]+msg.data[1])
            missingCells_data.append(msg.data[3])
        except Exception as e:
            print("Failed to append data: ", e)

# Create a figure and axes
fig, ax = plt.subplots()

# Plot data
ax.plot(time_data, occupiedCells_data, label='Occupied Cells')
ax.plot(time_data, freeCells_data, label='Free Cells')
ax.plot(time_data, totalCells_data, label='Total Cells')
ax.plot(time_data, missingCells_data, label='Missing Cells')

# Set labels and title
ax.set_xlabel('Time (s)')
ax.set_ylabel('Cell Count')
ax.set_title('Cell Counts vs Time')

# Add a legend
ax.legend()

# Show the plot
plt.show()
