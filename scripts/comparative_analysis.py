#!/usr/bin/env python

import rosbag
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt

# List of rosbag files
rosbag_files = ['/home/andre/thesis/lambda004.bag']

def extract_data_from_rosbag(rosbag_file):
    bag = rosbag.Bag(rosbag_file)
    data = {'time': [], 'free_cells': [], 'occupied_cells': [], 'unknown_cells': [], 'known_cells': []}

    for topic, msg, t in bag.read_messages(topics=['/uav1/results']):
        if topic == '/uav1/results':
            data['time'].append(msg.data[0])
            data['free_cells'].append(100 * msg.data[4] / msg.data[7])
            data['occupied_cells'].append(100 * msg.data[5] / msg.data[7])
            data['unknown_cells'].append(100 * msg.data[8] / msg.data[7])
            data['known_cells'].append(100 * (msg.data[7] - msg.data[6]) / msg.data[7])
        # else:
        #     print(f"Unexpected message: {msg}")

    bag.close()
    return data

# Extract data from each rosbag
for rosbag_file in rosbag_files:
    data = extract_data_from_rosbag(rosbag_file)
    
    # Plot cells in function of time
    plt.plot(data['time'], data['known_cells'], label='Known Cells')
    plt.plot(data['time'], data['unknown_cells'], label='Unknown Cells')
    plt.plot(data['time'], data['free_cells'], label='Free Cells')
    plt.plot(data['time'], data['occupied_cells'], label='Occupied Cells')

    plt.xlabel('Time')
    plt.ylabel('Number of Cells')
    plt.title('Cells in Function of Time')
    plt.legend()

    plt.show()
