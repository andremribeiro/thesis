#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <mrs_msgs/PathSrv.h>
#include <mrs_msgs/Reference.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <frontier_exploration/path_planning.hpp>
#include <queue>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <memory>

class PathPlanning {
public:
    PathPlanning() {
        ros::NodeHandle nh;
        octomap_sub = nh.subscribe<octomap_msgs::Octomap>("octomap_full", 10, &PathPlanning::octomapCallback, this);
        odom_sub = nh.subscribe("uav1/estimation_manager/odom_main", 10, &PathPlanning::odomCallback, this);
        best_frontier_sub = nh.subscribe<geometry_msgs::PointStamped>("best_frontier", 10, &PathPlanning::bestFrontierCallback, this);
        path_pub = nh.advertise<nav_msgs::Path>("astar_path", 10);
        path_planning_done = false;
        safety_distance = 1.0;
    }

    double heuristic(Node a, Node b)
    {
        return sqrt(pow(b.x - a.x, 2) + pow(b.y - a.y, 2) + pow(b.z - a.z, 2));
    }

    std::vector<Node> getNeighbors(Node node)
    {
        std::vector<Node> neighbors;
        for (int dx = -1; dx <= 1; ++dx) 
        {
            for (int dy = -1; dy <= 1; ++dy) 
            {
                for (int dz = -1; dz <= 1; ++dz) 
                {
                    if (dx != 0 || dy != 0 || dz != 0) 
                    {
                        neighbors.push_back(Node{node.x + dx, node.y + dy, node.z + dz});
                    }
                }
            }
        }
        return neighbors;
    }

    bool isTraversable(const octomap::OcTree& octree, const Node& node) 
    {
        // Convert Node to octomap::point3d
        octomap::point3d point(node.x, node.y, node.z);

        // Query octree for occupancy
        octomap::OcTreeNode* result = octree.search(point);

        if (result == NULL) 
        {
            // Node is not in the octree, assume it's traversable
            return true;
        } else 
        {
            // Node is in the octree, check occupancy
            return !octree.isNodeOccupied(result);
        }
    }

    bool isSafe(const octomap::OcTree& octree, const Node& node)
    {
        // Convert Node to octomap::point3d
        octomap::point3d point(node.x, node.y, node.z);

        std::vector<Node> neighbors;
        for (int dx = -safety_distance; dx <= safety_distance; ++dx) 
        {
            for (int dy = -safety_distance; dy <= safety_distance; ++dy) 
            {
                for (int dz = -safety_distance; dz <= safety_distance; ++dz) 
                {
                    if (dx != 0 || dy != 0 || dz != 0) 
                    {
                        neighbors.push_back(Node{node.x + dx, node.y + dy, node.z + dz});
                    }
                }
            }
        }

        // Check the occupancy of the neighboring nodes
        for (const Node& neighbor : neighbors) 
        {
            octomap::point3d check_point(neighbor.x, neighbor.y, neighbor.z);
            octomap::OcTreeNode* check_result = octree.search(check_point);
            if (check_result != NULL && octree.isNodeOccupied(check_result)) 
            {
                return false;
            }
        }
        return true;
    }

    std::vector<Node> aStar(octomap::OcTree& octree, Node start, Node goal) 
    {
        std::priority_queue<std::pair<double, Node>> frontier;
        frontier.push(std::make_pair(0.0, start));

        std::unordered_map<Node, Node> came_from;
        std::unordered_map<Node, double> cost_so_far;

        came_from[start] = start;
        cost_so_far[start] = 0.0;

        while (!frontier.empty()) 
        {
            Node current = frontier.top().second;
            frontier.pop();

            if (current == goal) 
            {
                break;
            }

            for (Node neighbor : getNeighbors(current)) 
            {
                if (!isTraversable(octree, neighbor) || !isSafe(octree, neighbor)) continue;

                double new_cost = cost_so_far[current] + heuristic(current, neighbor);
                if (!cost_so_far.count(neighbor) || new_cost < cost_so_far[neighbor]) 
                {
                    cost_so_far[neighbor] = new_cost;
                    double priority = new_cost + heuristic(goal, neighbor);
                    frontier.push(std::make_pair(-priority, neighbor));
                    came_from[neighbor] = current;
                }
            }
        }

        // Reconstruct path
        std::vector<Node> path;
        Node current = goal;
        while (!(current == start)) 
        {
            path.push_back(current);
            current = came_from[current];
        }
        path.push_back(start); // optional
        std::reverse(path.begin(), path.end());

        return path;
    }

    void bestFrontierCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        goal_position.x() = msg->point.x;
        goal_position.y() = msg->point.y;
        goal_position.z() = msg->point.z;

        // Convert goal_position and current_position to Node
        Node start = {static_cast<int>(current_position.x() / octree->getResolution()), static_cast<int>(current_position.y() / octree->getResolution()), static_cast<int>(current_position.z() / octree->getResolution())};
        Node goal = {static_cast<int>(goal_position.x() / octree->getResolution()), static_cast<int>(goal_position.y() / octree->getResolution()), static_cast<int>(goal_position.z() / octree->getResolution())};

        // Call A* algorithm
        std::vector<Node> path = aStar(*octree.get(), start, goal);

        nav_msgs::Path path_msg;
        path_msg.header.stamp = ros::Time::now();
        path_msg.header.frame_id = "uav1/world_origin";

        for (const Node& node : path) 
        {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = "uav1/world_origin";  
            pose.pose.position.x = node.x * octree->getResolution();
            pose.pose.position.y = node.y * octree->getResolution();
            pose.pose.position.z = node.z * octree->getResolution();
            path_msg.poses.push_back(pose);
        }
        path_pub.publish(path_msg);

        ROS_INFO("Best frontier updated");
    }

    void odomCallback(const nav_msgs::OdometryConstPtr& msg)
    {
        current_position.x() = msg->pose.pose.position.x;
        current_position.y() = msg->pose.pose.position.y;
        current_position.z() = msg->pose.pose.position.z;
    }

    void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) 
    {
        octree.reset(dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg)));

        if (!octree) 
        {
            ROS_ERROR("Failed to convert Octomap message to OcTree");
            return;
        }

        ROS_INFO("Octomap updated");
    }

private:
    ros::Subscriber octomap_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber best_frontier_sub;
    ros::Publisher path_pub;
    ros::ServiceClient path_srv;
    octomap::point3d current_position;
    octomap::point3d goal_position;
    std::unique_ptr<octomap::OcTree> octree;
    bool path_planning_done;
    float safety_distance;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_planning");

    PathPlanning path_planning;

    ros::spin();

    return 0;
}
