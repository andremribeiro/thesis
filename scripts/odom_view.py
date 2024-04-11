#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import rosbag

class OdometryPathCreator:
    def __init__(self, bag_file):
        rospy.init_node('odometry_path_creator')
        self.odom_paths = {}  # Dictionary to store paths for each odometry topic
        self.bag = rosbag.Bag(bag_file)
        self.process_bag()

    def process_bag(self):
        for topic, msg, t in self.bag.read_messages(topics=['/uav1/estimation_manager/odom_main', '/uav2/estimation_manager/odom_main']):
            if topic not in self.odom_paths:
                self.odom_paths[topic] = Path()
                self.odom_paths[topic].header.frame_id = "common_origin"  # Change the frame ID if needed
            pose_stamped = PoseStamped()
            pose_stamped.pose = msg.pose.pose
            pose_stamped.header = msg.header
            self.odom_paths[topic].poses.append(pose_stamped)

        self.publish_paths()

    def publish_paths(self):
        for topic, path in self.odom_paths.items():
            path_pub = rospy.Publisher('/{}_path'.format(topic.split('/')[-1]), Path, queue_size=10)
            path_pub.publish(path)

if __name__ == '__main__':
    try:
        bag_file = "/home/andre/thesis/sims/2_uav/test4.bag" # Specify your rosbag file
        OdometryPathCreator(bag_file)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
