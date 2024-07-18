#!/usr/bin/env python

import rosbag
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt

# List of rosbag files
rosbag_files = ['/home/andre/thesis/bags/other/multi_uav/test_3uav_collab_1.bag',
                '/home/andre/thesis/bags/other/multi_uav/test_3uav_low_comms_4.bag',
                '/home/andre/thesis/bags/3_uav/test_proposed_1.bag']

def extract_uav_data_from_rosbag(rosbag_file, num_uavs):
    bag = rosbag.Bag(rosbag_file, 'r')
    uav_data = [{'time': [], 'known_cells': []} for _ in range(num_uavs)]  # Only extract required fields
    first_timestamp = None

    for topic, msg, _ in bag.read_messages(topics=[f'/uav{i+1}/results' for i in range(num_uavs)]):
        uav_index = int(topic[4]) - 1

        if first_timestamp is None:
            first_timestamp = msg.data[0]
        if msg.data[0] - first_timestamp > 0:
            uav_data[uav_index]['time'].append(msg.data[0] - first_timestamp)
            uav_data[uav_index]['known_cells'].append(100 * (msg.data[7] - msg.data[6]) / msg.data[7])

    bag.close()
    return uav_data


lambdas = ['Map-Merging', 'DSGA', 'Proposed']

for i, rosbag_file in enumerate(rosbag_files):
    num_uavs = 2 if ('2_uav' in rosbag_file or '2uav' in rosbag_file) else 3
    uav_data = extract_uav_data_from_rosbag(rosbag_file, num_uavs)

    # Find the max coverage at each timestamp and ensure non-decreasing coverage
    combined_time = sorted(set(t for uav in uav_data for t in uav['time']))
    combined_coverage = []
    print('First coverage per UAV:', [uav['known_cells'][0] for uav in uav_data])

    max_coverage_so_far = 0
    for t in combined_time:
        max_coverage_at_t = max(
            uav['known_cells'][uav['time'].index(t)]
            for uav in uav_data 
            if t in uav['time']
        )
        max_coverage_so_far = max(max_coverage_so_far, max_coverage_at_t)
        combined_coverage.append(max_coverage_so_far)

    plt.plot(combined_time, combined_coverage, label=f'{lambdas[i]}')

plt.xlabel('Exploration Time (s)')
plt.ylabel('Coverage (%)')
plt.title('Environment Coverage in Function of Exploration Time')
plt.legend()
plt.grid()
plt.ylim(0, 100)

plt.show()
