#!/usr/bin/env python

import rosbag
from std_msgs.msg import Float64MultiArray
import numpy as np
import matplotlib.pyplot as plt


# List of rosbag files
rosbag_files = ['/home/andre/thesis/bags/multi_uav/forest_3uav_low_comms_1.bag', 
                '/home/andre/thesis/bags/multi_uav/forest_3uav_low_comms_2.bag', 
                '/home/andre/thesis/bags/multi_uav/forest_3uav_low_comms_3.bag', 
                '/home/andre/thesis/bags/multi_uav/forest_3uav_low_comms_4.bag', 
                '/home/andre/thesis/bags/multi_uav/forest_3uav_low_comms_5.bag']

def extract_uav_data_from_rosbag(rosbag_file, num_uavs):
    bag = rosbag.Bag(rosbag_file, 'r')
    uav_data = [{'time': [], 'position_x': [], 'position_y': [], 'position_z': [], 'free_cells': [], 'occupied_cells': [], 'unknown_cells': [], 'known_cells': [], 'frontier_cells': [], 'frontier_candidates': [], 'waypoint_x': [], 'waypoint_y': [], 'waypoint_z': [], 'waypoint_score': [], 'detection_time': [], 'clustering_time': [], 'evaluation_time': [], 'computation_time': []} for _ in range(num_uavs)]  # Initialize uav_data list with empty dictionaries
    first_timestamp = None
    prev_positions = [None] * num_uavs
    path_lengths = [0] * num_uavs

    for topic, msg, _ in bag.read_messages(topics=[f'/uav{i+1}/results' for i in range(num_uavs)]):
        uav_index = int(topic[4]) - 1

        if first_timestamp is None:
            first_timestamp = msg.data[0]

        uav_data[uav_index]['time'].append(msg.data[0] - first_timestamp)
        uav_data[uav_index]['position_x'].append(msg.data[1])
        uav_data[uav_index]['position_y'].append(msg.data[2])
        uav_data[uav_index]['position_z'].append(msg.data[3])

        curr_position = np.array([msg.data[1], msg.data[2], msg.data[3]])
        if prev_positions[uav_index] is not None:
            path_lengths[uav_index] += np.linalg.norm(curr_position - prev_positions[uav_index])
        prev_positions[uav_index] = curr_position

        uav_data[uav_index]['free_cells'].append(100 * msg.data[4] / msg.data[7])
        uav_data[uav_index]['occupied_cells'].append(100 * msg.data[5] / msg.data[7])
        uav_data[uav_index]['unknown_cells'].append(100 * msg.data[6] / msg.data[7])
        uav_data[uav_index]['known_cells'].append(100 * (msg.data[7] - msg.data[6]) / msg.data[7])
        uav_data[uav_index]['frontier_cells'].append(msg.data[8])
        uav_data[uav_index]['frontier_candidates'].append(msg.data[9])
        uav_data[uav_index]['waypoint_x'].append(msg.data[10])
        uav_data[uav_index]['waypoint_y'].append(msg.data[11])
        uav_data[uav_index]['waypoint_z'].append(msg.data[12])
        uav_data[uav_index]['waypoint_score'].append(msg.data[13])
        uav_data[uav_index]['detection_time'].append(msg.data[15])
        uav_data[uav_index]['clustering_time'].append(msg.data[16])
        uav_data[uav_index]['evaluation_time'].append(msg.data[17])
        uav_data[uav_index]['computation_time'].append(msg.data[15] + msg.data[16] + msg.data[17])

    bag.close()
    return uav_data, path_lengths


results = []
exploration_rate = []
path_efficiency = []
coverage_thresholds = [50, 75, 90]

if rosbag_files[0].startswith('test'):
    volume = 14400
else:
    volume = 32400

time_to_reach_coverage = []
detection_times = []
clustering_times = []
evaluation_times = []
computation_times = []

for rosbag_file in rosbag_files:
    num_uavs = 2 if '2uav' in rosbag_file else 3
    uav_data, path_lengths = extract_uav_data_from_rosbag(rosbag_file, num_uavs)

    print(uav_data[0]['known_cells'][-1])
    print(uav_data[1]['known_cells'][-1])
    if num_uavs == 3:
        print(uav_data[2]['known_cells'][-1])
    print(uav_data[0]['position_z'][-1])
    print(uav_data[1]['position_z'][-1])
    if num_uavs == 3:
        print(uav_data[2]['position_z'][-1])

    results.append((uav_data, path_lengths))

time_values = [np.max([uav_data['time'][-1] for uav_data in uav_data_list]) for uav_data_list, _ in results]
time_average = np.mean(time_values)
time_std = np.std(time_values)
time_min = np.min(time_values)
time_max = np.max(time_values)

uav_path_length_values = [np.mean(path_lengths) for _, path_lengths in results]
uav_path_length_average = np.mean(uav_path_length_values)
uav_path_length_std = np.std(uav_path_length_values)
uav_path_length_min = np.min(uav_path_length_values)
uav_path_length_max = np.max(uav_path_length_values)

coverage_values = [np.max([uav_data['known_cells'][-1] for uav_data in uav_data_list]) for uav_data_list, _ in results]
coverage_average = np.mean(coverage_values)
coverage_std = np.std(coverage_values)
coverage_min = np.min(coverage_values)
coverage_max = np.max(coverage_values)

for i, (uav_data_list, path_lengths) in enumerate(results):
    exploration_rate.append(np.max([uav_data['known_cells'][-1] for uav_data in uav_data_list]) * volume / (100 * np.max([uav_data['time'][-1] for uav_data in uav_data_list])))
    path_efficiency.append(np.max([uav_data['known_cells'][-1] for uav_data in uav_data_list]) * volume / (100 * np.sum(path_lengths)))
    # detection_times.append(1000 * np.mean(uav1_data['detection_time']))
    # clustering_times.append(1000 * np.mean(uav1_data['clustering_time']))
    # evaluation_times.append(1000 * np.mean(uav1_data['evaluation_time']))
    # computation_times.append(1000 * np.mean(uav1_data['computation_time']))

print(exploration_rate)
print(path_efficiency)

# Calculate statistics for exploration rate
exploration_rate_average = np.mean(exploration_rate)
exploration_rate_std = np.std(exploration_rate)
exploration_rate_min = np.min(exploration_rate)
exploration_rate_max = np.max(exploration_rate)

path_efficiency_average = np.mean(path_efficiency)
path_efficiency_std = np.std(path_efficiency)
path_efficiency_min = np.min(path_efficiency)
path_efficiency_max = np.max(path_efficiency)

# Calculate time to reach coverage thresholds
time_to_reach_coverage_matrix = np.zeros((len(results), len(coverage_thresholds)))

for i, (uav_data_list, _) in enumerate(results):
    for j, threshold in enumerate(coverage_thresholds):
        earliest_time = float('inf')
        for uav_data in uav_data_list:
            coverage = uav_data['known_cells']
            time = uav_data['time']
            for k, coverage_value in enumerate(coverage):
                if coverage_value >= threshold:
                    earliest_time = min(earliest_time, time[k])
                    break
        time_to_reach_coverage_matrix[i, j] = earliest_time if earliest_time != float('inf') else 0

time_to_reach_coverage_avg_50 = np.mean(time_to_reach_coverage_matrix[:, 0][time_to_reach_coverage_matrix[:, 0] > 0])
time_to_reach_coverage_std_50 = np.std(time_to_reach_coverage_matrix[:, 0][time_to_reach_coverage_matrix[:, 0] > 0])
time_to_reach_coverage_min_50 = np.min(time_to_reach_coverage_matrix[:, 0][time_to_reach_coverage_matrix[:, 0] > 0])
time_to_reach_coverage_max_50 = np.max(time_to_reach_coverage_matrix[:, 0][time_to_reach_coverage_matrix[:, 0] > 0])

time_to_reach_coverage_avg_75 = np.mean(time_to_reach_coverage_matrix[:, 1][time_to_reach_coverage_matrix[:, 1] > 0])
time_to_reach_coverage_std_75 = np.std(time_to_reach_coverage_matrix[:, 1][time_to_reach_coverage_matrix[:, 1] > 0])
time_to_reach_coverage_min_75 = np.min(time_to_reach_coverage_matrix[:, 1][time_to_reach_coverage_matrix[:, 1] > 0])
time_to_reach_coverage_max_75 = np.max(time_to_reach_coverage_matrix[:, 1][time_to_reach_coverage_matrix[:, 1] > 0])

time_to_reach_coverage_avg_90 = np.mean(time_to_reach_coverage_matrix[:, 2][time_to_reach_coverage_matrix[:, 2] > 0])
time_to_reach_coverage_std_90 = np.std(time_to_reach_coverage_matrix[:, 2][time_to_reach_coverage_matrix[:, 2] > 0])
time_to_reach_coverage_min_90 = np.min(time_to_reach_coverage_matrix[:, 2][time_to_reach_coverage_matrix[:, 2] > 0])
time_to_reach_coverage_max_90 = np.max(time_to_reach_coverage_matrix[:, 2][time_to_reach_coverage_matrix[:, 2] > 0])
    
# detection_time_avg = np.mean(detection_times)
# detection_time_std = np.std(detection_times)
# detection_time_min = np.min(detection_times)
# detection_time_max = np.max(detection_times)

# clustering_time_avg = np.mean(clustering_times)
# clustering_time_std = np.std(clustering_times)
# clustering_time_min = np.min(clustering_times)
# clustering_time_max = np.max(clustering_times)

# evaluation_time_avg = np.mean(evaluation_times)
# evaluation_time_std = np.std(evaluation_times)
# evaluation_time_min = np.min(evaluation_times)
# evaluation_time_max = np.max(evaluation_times)

# computation_times_avg = np.mean(computation_times)
# computation_times_std = np.std(computation_times)
# computation_times_min = np.min(computation_times)
# computation_times_max = np.max(computation_times)

print("Time - Average: {:.2f}".format(time_average))
print("Time - Standard Deviation: {:.2f}".format(time_std))
print("Time - Minimum: {:.2f}".format(time_min))
print("Time - Maximum: {:.2f}".format(time_max))

print("Path Length - Average: {:.2f}".format(uav_path_length_average))
print("Path Length - Standard Deviation: {:.2f}".format(uav_path_length_std))
print("Path Length - Minimum: {:.2f}".format(uav_path_length_min))
print("Path Length - Maximum: {:.2f}".format(uav_path_length_max))

print("Coverage - Average: {:.2f}".format(coverage_average))
print("Coverage - Standard Deviation: {:.2f}".format(coverage_std))
print("Coverage - Minimum: {:.2f}".format(coverage_min))
print("Coverage - Maximum: {:.2f}".format(coverage_max))

print("Exploration Rate - Average: {:.2f}".format(exploration_rate_average))
print("Exploration Rate - Standard Deviation: {:.2f}".format(exploration_rate_std))
print("Exploration Rate - Minimum: {:.2f}".format(exploration_rate_min))
print("Exploration Rate - Maximum: {:.2f}".format(exploration_rate_max))

print("Path Efficiency - Average: {:.2f}".format(path_efficiency_average))
print("Path Efficiency - Standard Deviation: {:.2f}".format(path_efficiency_std))
print("Path Efficiency - Minimum: {:.2f}".format(path_efficiency_min))
print("Path Efficiency - Maximum: {:.2f}".format(path_efficiency_max))

print("Time to reach 50 coverage - Average: {:.2f}".format(time_to_reach_coverage_avg_50))
print("Time to reach 50 coverage - Standard Deviation: {:.2f}".format(time_to_reach_coverage_std_50))
print("Time to reach 50 coverage - Minimum: {:.2f}".format(time_to_reach_coverage_min_50))
print("Time to reach 50 coverage - Maximum: {:.2f}".format(time_to_reach_coverage_max_50))

print("Time to reach 75 coverage - Average: {:.2f}".format(time_to_reach_coverage_avg_75))
print("Time to reach 75 coverage - Standard Deviation: {:.2f}".format(time_to_reach_coverage_std_75))
print("Time to reach 75 coverage - Minimum: {:.2f}".format(time_to_reach_coverage_min_75))
print("Time to reach 75 coverage - Maximum: {:.2f}".format(time_to_reach_coverage_max_75))

print("Time to reach 90 coverage - Average: {:.2f}".format(time_to_reach_coverage_avg_90))
print("Time to reach 90 coverage - Standard Deviation: {:.2f}".format(time_to_reach_coverage_std_90))
print("Time to reach 90 coverage - Minimum: {:.2f}".format(time_to_reach_coverage_min_90))
print("Time to reach 90 coverage - Maximum: {:.2f}".format(time_to_reach_coverage_max_90))

# print("Detection Time - Average: {:.2f}".format(detection_time_avg))
# print("Detection Time - Standard Deviation: {:.2f}".format(detection_time_std))
# # print("Detection Time - Minimum: {:.2f}".format(detection_time_min))
# # print("Detection Time - Maximum: {:.2f}".format(detection_time_max))

# print("Clustering Time - Average: {:.2f}".format(clustering_time_avg))
# print("Clustering Time - Standard Deviation: {:.2f}".format(clustering_time_std))
# # print("Clustering Time - Minimum: {:.2f}".format(clustering_time_min))
# # print("Clustering Time - Maximum: {:.2f}".format(clustering_time_max))

# print("Evaluation Time - Average: {:.2f}".format(evaluation_time_avg))
# print("Evaluation Time - Standard Deviation: {:.2f}".format(evaluation_time_std))
# # print("Evaluation Time - Minimum: {:.2f}".format(evaluation_time_min))
# # print("Evaluation Time - Maximum: {:.2f}".format(evaluation_time_max))

# print("Computation Time - Average: {:.2f}".format(computation_times_avg))
# print("Computation Time - Standard Deviation: {:.2f}".format(computation_times_std))
# # print("Computation Time - Minimum: {:.2f}".format(computation_times_min))
# # print("Computation Time - Maximum: {:.2f}".format(computation_times_max))