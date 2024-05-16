#!/usr/bin/env python

import rosbag
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt

# List of rosbag files
rosbag_files = ['/home/andre/thesis/bags/lambda/test_40_5.bag', '/home/andre/thesis/bags/lambda/test_30_3.bag', '/home/andre/thesis/bags/lambda/test_20_3.bag', '/home/andre/thesis/bags/lambda/test_10_1.bag']

def extract_data_from_rosbag(rosbag_file):
    bag = rosbag.Bag(rosbag_file)
    data = {'time': [], 'free_cells': [], 'occupied_cells': [], 'unknown_cells': [], 'known_cells': []}

    first_timestamp = None

    for topic, msg, t in bag.read_messages(topics=['/uav1/results']):
        if topic == '/uav1/results':
            if msg.data[8] > 0 or msg.data[-1] > 0.5:
                if first_timestamp is None:
                    first_timestamp = msg.data[0]

                data['time'].append(msg.data[0] - first_timestamp)
                data['free_cells'].append(100 * msg.data[4] / msg.data[7])
                data['occupied_cells'].append(100 * msg.data[5] / msg.data[7])
                data['unknown_cells'].append(100 * msg.data[6] / msg.data[7])
                data['known_cells'].append(100 * (msg.data[7] - msg.data[6]) / msg.data[7])
        # else:
        #     print(f"Unexpected message: {msg}")

    bag.close()
    return data

# Extract data from each rosbag
all_data = []

lambdas = [1.0, 2.0, 3.0, 4.0]
for i, rosbag_file in enumerate(rosbag_files):
    data = extract_data_from_rosbag(rosbag_file)
    all_data.append(data)

    # Plot cells in function of time for each rosbag
    plt.plot(data['time'], data['known_cells'], label=f'lambda={lambdas[i]}')

plt.xlabel('Time (s)')
plt.ylabel('Mapped Cells (%)')
plt.title('Mapped Cells in Function of Time')
plt.legend()

plt.show()
