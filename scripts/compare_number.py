import matplotlib.pyplot as plt
import numpy as np

# Number of UAVs
uavs = np.array([1, 2, 3])

# Time taken for exploration
time_taken_test = np.array([85.68, 36.01, 36.73])
time_taken_forest = np.array([225.37, 108.33, 84.08])

# Standard deviation
time_std_dev_test = np.array([18.26, 4.57, 0.99])
time_std_dev_forest = np.array([36.54, 3.43, 5.38])

path_length_test = np.array([141.79, 62.56 , 65.44])
path_length_forest = np.array([376.66, 187.94, 161.47])

path_std_dev_test = np.array([18.83 , 3.03, 2.99])
path_std_dev_forest = np.array([55.10, 5.02, 11.44])

# Mapping coverage
coverage_test = np.array([92.54, 91.15, 93.29])
coverage_forest = np.array([95.37, 92.90, 96.92])

coverage_std_dev_test = np.array([1.41 , 0.92, 0.61])
coverage_std_dev_forest = np.array([2.54, 1.02, 1.32])
# Create a figure and a set of subplots for test results
fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 5))

# Plot the averages with standard deviation as fill between for Time Taken
ax1.plot(uavs, time_taken_test, 'o-', label='Test')
ax1.fill_between(uavs, time_taken_test - time_std_dev_test, time_taken_test + time_std_dev_test, alpha=0.3)
ax1.set_xlabel('Number of UAVs')
ax1.set_ylabel('Exlporation Time (s)')
ax1.grid()

# Plot the path lengths with standard deviation as fill between
ax2.plot(uavs, path_length_test, 'o-', label='Test')
ax2.fill_between(uavs, path_length_test - path_std_dev_test, path_length_test + path_std_dev_test, alpha=0.3)
ax2.set_xlabel('Number of UAVs')
ax2.set_ylabel('Path Length (m)')
ax2.grid()

# Plot the mapping coverage with standard deviation as fill between
ax3.plot(uavs, coverage_test, 'o-', label='Test')
ax3.fill_between(uavs, coverage_test - coverage_std_dev_test, coverage_test + coverage_std_dev_test, alpha=0.3)
ax3.set_xlabel('Number of UAVs')
ax3.set_ylabel('Mapping Coverage (%)')
ax3.grid()

# Adjust the spacing between subplots
plt.tight_layout()

# Show the plot for test results
plt.show()

# Create a figure and a set of subplots for forest results
fig, (ax4, ax5, ax6) = plt.subplots(1, 3, figsize=(15, 5))

# Plot the averages with standard deviation as fill between for Time Taken
ax4.plot(uavs, time_taken_forest, 'o-', label='Forest')
ax4.fill_between(uavs, time_taken_forest - time_std_dev_forest, time_taken_forest + time_std_dev_forest, alpha=0.3)
ax4.set_xlabel('Number of UAVs')
ax4.set_ylabel('Exlporation Time (s)')
ax4.grid()

# Plot the path lengths with standard deviation as fill between
ax5.plot(uavs, path_length_forest, 'o-', label='Forest')
ax5.fill_between(uavs, path_length_forest - path_std_dev_forest, path_length_forest + path_std_dev_forest, alpha=0.3)
ax5.set_xlabel('Number of UAVs')
ax5.set_ylabel('Path Length (m)')
ax5.grid()

# Plot the mapping coverage with standard deviation as fill between
ax6.plot(uavs, coverage_forest, 'o-', label='Forest')
ax6.fill_between(uavs, coverage_forest - coverage_std_dev_forest, coverage_forest + coverage_std_dev_forest, alpha=0.3)
ax6.set_xlabel('Number of UAVs')
ax6.set_ylabel('Mapping Coverage (%)')
ax6.grid()

# Adjust the spacing between subplots
plt.tight_layout()

# Show the plot for forest results
plt.show()
