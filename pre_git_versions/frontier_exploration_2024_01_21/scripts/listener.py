#!/usr/bin/env python

import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray
from mrs_msgs.srv import Vec4

free_cells = []
occupied_cells = []

current_position = None

def callback_free(data):
    global free_cells
    markers = data.markers
    for marker in markers:
        points = marker.points
        for point in points:
            free_cells.append((point.x, point.y, point.z))

def callback_occupied(data):
    global occupied_cells
    markers = data.markers
    for marker in markers:
        points = marker.points
        for point in points:
            occupied_cells.append((point.x, point.y, point.z))

def callback_odom(data):
    global current_position
    current_position = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])

def is_unknown(cell):
    global free_cells, occupied_cells
    return cell not in free_cells and cell not in occupied_cells

def is_occupied(cell):
    global occupied_cells
    return cell in occupied_cells

def get_neighbors(cell, size):
    x, y, z = cell
    neighbors = [(x+dx, y+dy, z+dz) for dx in range(-size, size+1) for dy in range(-size, size+1) for dz in range(-size, size+1) if not (dx == 0 and dy == 0 and dz == 0)]
    return neighbors

def calculate_score(cell, lambda_constant):
    global current_position
    neighbors = get_neighbors(cell, 2)
    unknown_neighbors = {neighbor for neighbor in neighbors if is_unknown(neighbor)}
    information_gain = len(unknown_neighbors) / len(neighbors)
    print(information_gain)
    distance = np.linalg.norm(np.array(cell) - current_position)
    score = information_gain * math.exp(-lambda_constant * distance)
    return score

def find_best_frontier():
    global free_cells
    lambda_constant = 0.00025
    best_score = -math.inf
    best_frontier = None
    for cell in free_cells:
        neighbors = get_neighbors(cell, 1)
        if any(is_occupied(neighbor) for neighbor in neighbors):
            continue
        if any(is_unknown(neighbor) for neighbor in neighbors):
            score = calculate_score(cell, lambda_constant)
            if score > best_score:
                best_score = score
                best_frontier = cell
    
    if best_frontier is not None:
        print("Best frontier found at cell: ", best_frontier)
        print("Best score: ", best_score)
        # goto_service = rospy.ServiceProxy('/uav1/octomap_planner/goto', Vec4)
        # goal = list(best_frontier) + [0.0]
        # response = goto_service(goal)
        # print("Service response: ", response)
    else:
        print("No frontier found")

    return best_frontier

def listener():
    rospy.init_node('frontier_exploration', anonymous=True)
    rospy.Subscriber("/free_cells_vis_array", MarkerArray, callback_free)
    rospy.Subscriber("/occupied_cells_vis_array", MarkerArray, callback_occupied)
    rospy.Subscriber("/uav1/estimation_manager/odom_main", Odometry, callback_odom)
    rate = rospy.Rate(10) # 10Hz
    while not rospy.is_shutdown():
        find_best_frontier()
        rate.sleep()

if __name__ == '__main__':
    listener()
