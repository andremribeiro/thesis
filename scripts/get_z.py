import rosbag
import numpy as np

import matplotlib.pyplot as plt

# List of rosbag file paths
# List of rosbag files
rosbag_files = ['/home/andre/thesis/bags/placeholder/single_uav/test_10_1.bag']

# Initialize empty lists for x and y positions
z_position = []
velocity = []
time_odom = []
first_timestamp = None

# Iterate over each rosbag file
for rosbag_file in rosbag_files:
    # Open the rosbag file
    bag = rosbag.Bag(rosbag_file)

    # Iterate over each message in the rosbag
    for topic, msg, t in bag.read_messages(topics=['/uav1/estimation_manager/odom_main']):
        # Extract the x and y positions from the odom message
        if first_timestamp is None:
            first_timestamp = t.to_sec()
        time_odom.append(t.to_sec()-first_timestamp)
        z_position.append(msg.pose.pose.position.z)
        velocity.append(np.linalg.norm([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]))

    # Close the rosbag file
    bag.close()

    # Plot height vs time
    plt.figure()
    plt.plot(time_odom, z_position)
    plt.xlabel('Exploration Time (s)')
    plt.ylabel('Flight Height (m)')
    plt.title('Flight Height in Function of Exploration Time')
    plt.grid()

    # Plot speed vs time
    plt.figure()
    plt.plot(time_odom, velocity)
    plt.xlabel('Exploration Time (s)')
    plt.ylabel('Flight Speed (m/s)')
    plt.title('Flight Speed in Function of Exploration Time')
    plt.grid()

    # Show the plots
    plt.show()