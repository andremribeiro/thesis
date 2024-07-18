#!/usr/bin/env python

import rosbag
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict

# List of rosbag files
rosbag_files = ['/home/andre/thesis/bags/1_uav/forest_bw1_1.bag', 
                '/home/andre/thesis/bags/1_uav/forest_bw1_2.bag', 
                '/home/andre/thesis/bags/1_uav/forest_bw1_3.bag', 
                '/home/andre/thesis/bags/1_uav/forest_bw1_4.bag', 
                '/home/andre/thesis/bags/1_uav/forest_bw1_5.bag',]

def extract_data_from_rosbag(rosbag_file):
    bag = rosbag.Bag(rosbag_file)
    data = {'time': [], 'position_x': [], 'position_y': [], 'position_z': [], 'free_cells': [], 'occupied_cells': [], 'unknown_cells': [], 'known_cells': [], 'frontier_cells': [], 'frontier_candidates': [], 'waypoint_x': [], 'waypoint_y': [], 'waypoint_z': [], 'waypoint_score': [], 'detection_time': [], 'clustering_time': [], 'evaluation_time': []}

    first_timestamp = None
    prev_position = None
    path_length = 0

    for topic, msg, t in bag.read_messages(topics=['/uav1/results']):
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
    bag.close()
    return data, path_length

# Initialize a dictionary to hold aggregated data
aggregated_data = defaultdict(lambda: {'detection_time': [], 'clustering_time': [], 'evaluation_time': [], 'total_time': []})

# Loop through each rosbag file and extract data
for rosbag_file in rosbag_files:
    data, path_length = extract_data_from_rosbag(rosbag_file)

    # Aggregate data by number of frontier cells
    for i in range(len(data['frontier_cells'])):
        frontier_cells = data['frontier_cells'][i]
        detection_time = data['detection_time'][i]
        clustering_time = data['clustering_time'][i]
        evaluation_time = data['evaluation_time'][i]
        total_time = detection_time + clustering_time + evaluation_time

        aggregated_data[frontier_cells]['detection_time'].append(detection_time)
        aggregated_data[frontier_cells]['clustering_time'].append(clustering_time)
        aggregated_data[frontier_cells]['evaluation_time'].append(evaluation_time)
        aggregated_data[frontier_cells]['total_time'].append(total_time)

# Calculate average times and standard deviations
frontier_cells = sorted(aggregated_data.keys())
avg_detection_time = [np.mean(aggregated_data[fc]['detection_time']) * 1000 for fc in frontier_cells]  # Convert to ms
std_detection_time = [np.std(aggregated_data[fc]['detection_time']) * 1000 for fc in frontier_cells]   # Convert to ms
avg_clustering_time = [np.mean(aggregated_data[fc]['clustering_time']) * 1000 for fc in frontier_cells]  # Convert to ms
std_clustering_time = [np.std(aggregated_data[fc]['clustering_time']) * 1000 for fc in frontier_cells]   # Convert to ms
avg_evaluation_time = [np.mean(aggregated_data[fc]['evaluation_time']) * 1000 for fc in frontier_cells]  # Convert to ms
std_evaluation_time = [np.std(aggregated_data[fc]['evaluation_time']) * 1000 for fc in frontier_cells]   # Convert to ms
avg_total_time = [np.mean(aggregated_data[fc]['total_time']) * 1000 for fc in frontier_cells]  # Convert to ms
std_total_time = [np.std(aggregated_data[fc]['total_time']) * 1000 for fc in frontier_cells]   # Convert to ms

# Plot computation times in function of number of frontier cells
fig, ax = plt.subplots()

# Plot average detection time with standard deviation
ax.plot(frontier_cells, avg_detection_time, label='Detection time')
ax.fill_between(frontier_cells, 
                np.array(avg_detection_time) - np.array(std_detection_time), 
                np.array(avg_detection_time) + np.array(std_detection_time), alpha=0.2)

# Plot average clustering time with standard deviation
ax.plot(frontier_cells, avg_clustering_time, label='Clustering time')
ax.fill_between(frontier_cells, 
                np.array(avg_clustering_time) - np.array(std_clustering_time), 
                np.array(avg_clustering_time) + np.array(std_clustering_time), alpha=0.2)

# Plot average evaluation time with standard deviation
ax.plot(frontier_cells, avg_evaluation_time, label='Evaluation time')
ax.fill_between(frontier_cells, 
                np.array(avg_evaluation_time) - np.array(std_evaluation_time), 
                np.array(avg_evaluation_time) + np.array(std_evaluation_time), alpha=0.2)

# Plot average total time with standard deviation
ax.plot(frontier_cells, avg_total_time, label='Total time')
ax.fill_between(frontier_cells, 
                np.array(avg_total_time) - np.array(std_total_time), 
                np.array(avg_total_time) + np.array(std_total_time), alpha=0.2)

ax.set_xlabel('Number of Frontier Cells')
ax.set_ylabel('Computation Time (ms)')
ax.legend()
ax.grid()
ax.set_title('Avg. Computation Time in Function of Number of Frontier Cells')
plt.show()
