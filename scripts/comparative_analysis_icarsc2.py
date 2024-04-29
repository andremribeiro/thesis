import matplotlib.pyplot as plt
import numpy as np

# Number of UAVs
uavs = np.array([1, 2, 3])

# Time taken for exploration
time_taken = np.array([223, 145.8, 87.8])

# Mapping coverage
mapping_coverage = np.array([98.6, 96.0, 98.6])

# Standard deviation
time_std_dev = np.array([13.9, 53.3, 10.1])
mapping_std_dev = np.array([0.8, 2.9, 0.4])

# Create a figure and a set of subplots
fig, ax = plt.subplots()

# Plot the data with error bars
ax.bar(uavs, time_taken, yerr=time_std_dev, width=0.25, color='cyan', capsize=4, ecolor='black')

# Set the labels for the x and y axes
ax.set_xlabel('Number of UAVs')
ax.set_ylabel('Exploration Time (s)')

# Set the title of the plot
ax.set_title('Exploration Results vs Number of UAVs')

ax2 = ax.twinx()
ax2.plot(uavs, mapping_coverage, color='red')

ax.set_ylabel('Mapping Coverage (%)')

# Display the plot
plt.show()
