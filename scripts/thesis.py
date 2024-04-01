#!/usr/bin/env python 

import rospy
from geometry_msgs.msg import PointStamped
from mrs_msgs.msg import Vec4
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray

from octomap_msgs.msg import Octomap
from octomap_msgs.msg import conversions
from octomap_msgs.srv import GetOctomap

from octomap_ros import conversions
from octomap import octomap
from octomap import OcTreeKey
from octomap import OcTree

import math
import visualization_msgs.msg

class UAV:
    def __init__(self, nh, uav_name):
        self.octomap_sub = rospy.Subscriber(uav_name + "/octomap_server/octomap_local_full", Octomap, self.octomapCallback)
        self.odom_sub = rospy.Subscriber(uav_name + "/estimation_manager/odom_main", Odometry, self.odomCallback)
        # self.planner_srv = rospy.ServiceProxy(uav_name + "/octomap_planner/goto", Vec4)
        # self.frontier_pub = rospy.Publisher(uav_name + "frontier_markers", visualization_msgs.msg.MarkerArray, queue_size=1)

        self.octree = None
        self.pos = None

        self.explorationMinX = -30
        self.explorationMaxX = 30
        self.explorationMinY = -30
        self.explorationMaxY = 30
        self.explorationMinZ = 1.0
        self.explorationMaxZ = 8.0

    def octomapCallback(self, msg):
        tree = conversions.binaryMsgToMap(msg)
        if tree:
            tree.expand()
            self.octree = tree
        else:
            rospy.logerr("Failed to convert octomap message to OcTree")

    def odomCallback(self, msg):
        self.pos = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]

class FrontierExploration:
    def __init__(self, num_uavs):
        self.uavs = []
        for i in range(num_uavs):
            uav_name = "uav" + str(i+1)
            self.uavs.append(UAV(rospy.NodeHandle(), uav_name))

    def run(self):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('multiuav_exploration', anonymous=True)
    num_uavs = 2
    fem = FrontierExploration(num_uavs)
    fem.run()
