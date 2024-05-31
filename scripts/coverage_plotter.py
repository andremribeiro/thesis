#!/usr/bin/env python

import rosbag
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
import numpy as np

# List of rosbag files
rosbag_file = '/home/andre/thesis/bags/1_uav/test_bw1_5.bag'

def extract_data_from_rosbag(rosbag_file):
    bag = rosbag.Bag(rosbag_file, 'r')
    data = {'time': [], 'position_x': [], 'position_y': [], 'position_z': [], 'free_cells': [], 'occupied_cells': [], 'unknown_cells': [], 'known_cells': [], 'frontier_cells': [], 'frontier_candidates': [], 'waypoint_x': [], 'waypoint_y': [], 'waypoint_z': [], 'waypoint_score': [], 'detection_time': [], 'clustering_time': [], 'evaluation_time': [], 'computation_time': []}

    first_timestamp = None
    prev_position = None
    path_length = 0

    for topic, msg, _ in bag.read_messages(topics=['/uav1/results']):
        if topic == '/uav1/results':
            if first_timestamp is None:
                first_timestamp = msg.data[0]

            data['time'].append(msg.data[0] - first_timestamp)
            data['position_x'].append(msg.data[1])
            data['position_y'].append(msg.data[2])
            data['position_z'].append(msg.data[3])

            curr_position = np.array([msg.data[1], msg.data[2], msg.data[3]])
            if prev_position is not None:
                path_length += np.linalg.norm(curr_position - prev_position)
            prev_position = curr_position

            data['free_cells'].append(100 * msg.data[4] / msg.data[7])
            data['occupied_cells'].append(100 * msg.data[5] / msg.data[7])
            data['unknown_cells'].append(100 * msg.data[6] / msg.data[7])
            data['known_cells'].append(100 * (msg.data[7] - msg.data[6]) / msg.data[7])
            data['frontier_cells'].append(msg.data[8])
            data['frontier_candidates'].append(msg.data[9])
            data['waypoint_x'].append(msg.data[10])
            data['waypoint_y'].append(msg.data[11])
            data['waypoint_z'].append(msg.data[12])
            data['waypoint_score'].append(msg.data[13])
            data['detection_time'].append(msg.data[15])
            data['clustering_time'].append(msg.data[16])
            data['evaluation_time'].append(msg.data[17])
            data['computation_time'].append(msg.data[15] + msg.data[16] + msg.data[17])
    bag.close()
    return data, path_length

data, path_length = extract_data_from_rosbag(rosbag_file)

plt.plot(data['time'], data['free_cells'], label='Free Cells')
plt.plot(data['time'], data['occupied_cells'], label='Occupied Cells')
plt.plot(data['time'], data['unknown_cells'], label='Unknown Cells')
plt.plot(data['time'], data['known_cells'], label='Known Cells')

plt.xlabel('Exploration Time (s)')
plt.ylabel('Coverage (%)')
plt.title('Environment Coverage in Function of Exploration Time')
plt.legend()
plt.grid()
plt.ylim(0, 100)

plt.show()
