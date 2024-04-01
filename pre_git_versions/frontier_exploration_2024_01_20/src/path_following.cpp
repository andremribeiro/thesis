#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <mrs_msgs/Vec4.h>
#include <cmath>

ros::ServiceClient client;
geometry_msgs::PointStamped current_position;

void bestFrontierCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    mrs_msgs::Vec4 srv;
    srv.request.goal[0] = msg->point.x;
    srv.request.goal[1] = msg->point.y;
    srv.request.goal[2] = msg->point.z;
    srv.request.goal[3] = atan2(msg->point.y - current_position.point.y, msg->point.x - current_position.point.x) * 180 / M_PI;

    if (client.call(srv))
    {
        ROS_INFO("Service called successfully");
    }
    else
    {
        ROS_ERROR("Failed to call service");
    }
}

void odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    current_position.point.x = msg->pose.pose.position.x;
    current_position.point.y = msg->pose.pose.position.y;
    current_position.point.z = msg->pose.pose.position.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_following");
    ros::NodeHandle nh;
    client = nh.serviceClient<mrs_msgs::Vec4>("uav1/octomap_planner/goto");
    ros::Subscriber best_frontier = nh.subscribe<geometry_msgs::PointStamped>("best_frontier", 1, bestFrontierCallback);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("uav1/estimation_manager/odom_main", 1, odomCallback);
    ros::spin();
    return 0;
}
