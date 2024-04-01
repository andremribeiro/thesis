import rosbag
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot_pointcloud(msg):
    gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    points = list(gen)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    zs = [point[2] for point in points]

    ax.scatter(xs, ys, zs, s=1)

    plt.show()

def read_and_plot_rosbag(bagfile, topic):
    bag = rosbag.Bag(bagfile, "r")
    for _, msg, _ in bag.read_messages(topics=[topic]):
        plot_pointcloud(msg)
    bag.close()

# Replace 'test.bag' with your rosbag file name
read_and_plot_rosbag('/home/andre/thesis/test.bag', '/uav1/os_cloud_nodelet/points')
