import rosbag
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
import numpy as np

# List of rosbag files
rosbag_files = ['/home/andre/thesis/sims/1_uav/test1.bag', '/home/andre/thesis/sims/1_uav/test2.bag', '/home/andre/thesis/sims/1_uav/test3.bag', '/home/andre/thesis/sims/1_uav/test4.bag', '/home/andre/thesis/sims/1_uav/test5.bag']

def extract_data_from_rosbag(rosbag_file):
    bag = rosbag.Bag(rosbag_file)
    data = {'time': [], 'free_cells': [], 'occupied_cells': [], 'unknown_cells': [], 'known_cells': []}

    for topic, msg, t in bag.read_messages(topics=['/uav1/coverage']):
        if topic == '/uav1/coverage':
            data['time'].append(msg.data[0])
            data['occupied_cells'].append(100 * msg.data[1] / msg.data[4])
            data['free_cells'].append(100 * msg.data[2] / msg.data[4])
            data['unknown_cells'].append(100 * msg.data[3] / msg.data[4])
            data['known_cells'].append(100 * (msg.data[1] + msg.data[2]) / msg.data[4])

    bag.close()
    return data

# Find the maximum time across all rosbags
max_time = max(max(data['time']) for data in [all_known_cells, all_unknown_cells, all_free_cells, all_occupied_cells])

# Create a common time base from 0 to max_time
common_time = np.linspace(0, max_time, 1000)

# Initialize lists to store interpolated data from all rosbags
all_known_cells = []
all_unknown_cells = []
all_free_cells = []
all_occupied_cells = []

# Extract data from each rosbag
for rosbag_file in rosbag_files:
    data = extract_data_from_rosbag(rosbag_file)
    
    # Append data to the lists
    all_known_cells.append(data['known_cells'])
    all_unknown_cells.append(data['unknown_cells'])
    all_free_cells.append(data['free_cells'])
    all_occupied_cells.append(data['occupied_cells'])

# Convert lists to numpy arrays
all_known_cells = np.array(all_known_cells)
all_unknown_cells = np.array(all_unknown_cells)
all_free_cells = np.array(all_free_cells)
all_occupied_cells = np.array(all_occupied_cells)

# Calculate the average and standard deviation for each type of cell
avg_known_cells = np.mean(all_known_cells, axis=0)
std_known_cells = np.std(all_known_cells, axis=0)

avg_unknown_cells = np.mean(all_unknown_cells, axis=0)
std_unknown_cells = np.std(all_unknown_cells, axis=0)

avg_free_cells = np.mean(all_free_cells, axis=0)
std_free_cells = np.std(all_free_cells, axis=0)

avg_occupied_cells = np.mean(all_occupied_cells, axis=0)
std_occupied_cells = np.std(all_occupied_cells, axis=0)

# Plot cells in function of time
plt.errorbar(data['time'], avg_known_cells, yerr=std_known_cells, label='Known Cells')
plt.errorbar(data['time'], avg_unknown_cells, yerr=std_unknown_cells, label='Unknown Cells')
plt.errorbar(data['time'], avg_free_cells, yerr=std_free_cells, label='Free Cells')
plt.errorbar(data['time'], avg_occupied_cells, yerr=std_occupied_cells, label='Occupied Cells')

plt.xlabel('Time')
plt.ylabel('Number of Cells')
plt.title('Cells in Function of Time')
plt.legend()

plt.show()
