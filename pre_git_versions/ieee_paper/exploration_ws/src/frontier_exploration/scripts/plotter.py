#!/usr/bin/env python

import rosbag
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry

# Path to your bag file
bag_path = '/home/andre/thesis/2024-02-11-19-21-38.bag'

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

# Initialize variable to hold total distance
total_distance = 0

def collect_data(bag_path):
    # Initialize variable to hold total distance
    total_distance = 0

    # Open the bag file
    with rosbag.Bag(bag_path, 'r') as bag:
        append_odom_data = True
        first_timestamp_coverage = None
        prev_position = None
        for topic, msg, t in bag.read_messages(topics=['/uav1/estimation_manager/odom_main', '/mappingCoverage']):
            try:
                if topic == '/uav1/estimation_manager/odom_main' and append_odom_data:
                    # Append data to lists
                    time_data_odom.append(t.to_sec())
                    x_data.append(msg.pose.pose.position.x)
                    y_data.append(msg.pose.pose.position.y)
                    z_data.append(msg.pose.pose.position.z)
                    # Calculate distance
                    curr_position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
                    if prev_position is not None:
                        total_distance += np.linalg.norm(curr_position - prev_position)
                    prev_position = curr_position

                elif topic == '/mappingCoverage':
                    # Stop appending data to odom_main topic
                    append_odom_data = False
                    # Append data to lists
                    if msg.data[3] > 0.01:
                        if first_timestamp_coverage is None:
                            first_timestamp_coverage = msg.data[0]
                        time_data_coverage.append(msg.data[0] - first_timestamp_coverage)
                        occupiedCells_data.append(msg.data[1] * 100)
                        freeCells_data.append(msg.data[2] * 100)
                        mappedCells_data.append((msg.data[1] + msg.data[2]) * 100)
                        missingCells_data.append(msg.data[3] * 100)
                        append_odom_data = True
            except Exception as e:
                print("Failed to append data: ", e)

    # Print total distance
    print("Total distance travelled by the UAV: ", total_distance, "meters")

def plot_data():
    # Create a figure and axes
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot odometry data
    ax.scatter(x_data, y_data, z_data, s=1, c='blue')

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
    ax2.plot(time_data_coverage, mappedCells_data, label='Mapped Cells', color='blue')
    ax2.plot(time_data_coverage, freeCells_data, label='Free Cells', color='green')
    ax2.plot(time_data_coverage, occupiedCells_data, label='Occupied Cells', color='red')
    ax2.plot(time_data_coverage, missingCells_data, label='Unknown Cells', color='orange')

    # Set labels and title for cell counts
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Cell Count (%)')
    ax2.set_title('Mapping Coverage')

    # Add a legend for cell counts
    ax2.legend()

    # Show the cell counts plot
    plt.show(block=True)

if __name__ == "__main__":
    collect_data(bag_path)
    plot_data()
