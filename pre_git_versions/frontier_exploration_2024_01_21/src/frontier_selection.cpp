#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <cmath>

class FrontierSelection
{
public:
    FrontierSelection() : nh()
    {
        frontier_sub = nh.subscribe("frontier_cells", 1, &FrontierSelection::frontierCellsCallback, this);
        odom_sub = nh.subscribe("uav1/estimation_manager/odom_main", 1, &FrontierSelection::odomCallback, this);
        octomap_sub = nh.subscribe("uav1/octomap_server/octomap_global_full", 1, &FrontierSelection::octomapCallback, this);
        best_frontier_pub = nh.advertise<geometry_msgs::PointStamped>("best_frontier", 1, true);
    }

    double calculateInformationGain(const pcl::PointXYZ& point)
    {
        if (octree == nullptr) 
        {
            ROS_ERROR("Octree is not initialized");
            return 0;
        }

        // Define the size of cube (sensor range)
        double sensor_range = 10.0;
        double resolution = octree->getResolution();

        // Initialize counters for total voxels and unknown voxels
        int total_voxels = 0;
        int unknown_voxels = 0;

        // Iterate over all voxels in cube
        for (double x = point.x - sensor_range; x <= point.x + sensor_range; x += resolution)
        {
            for (double y = point.y - sensor_range; y <= point.y + sensor_range; y += resolution)
            {
                for (double z = point.z - sensor_range; z <= point.z + sensor_range; z += resolution)
                {
                    // Increase the total voxel count
                    total_voxels++;

                    octomap::OcTreeNode* node = octree->search(x, y, z);
                    if (node != NULL)
                    {
                        // Node is within the octree bounds
                        if (octree->isNodeOccupied(node)) {} // Node is occupied
                        else {} // Node is free
                    }
                    else
                    {
                        // Node is unknown
                        unknown_voxels++;
                    }
                }
            }
        }
        return (double)unknown_voxels / total_voxels;
    }

    double calculateDistance(const pcl::PointXYZ& point, const pcl::PointXYZ& current_position)
    {
        double dx = point.x - current_position.x;
        double dy = point.y - current_position.y;
        double dz = point.z - current_position.z;

        return sqrt(dx*dx + dy*dy + dz*dz);
    }
    
    double calculateCellScore(const pcl::PointXYZ& point)
    {
        double lambda = 0.0025;

        double information_gain = calculateInformationGain(point);

        double distance = calculateDistance(point, current_position);

        return information_gain / exp(lambda * distance) * current_position.z;
    }

    void frontierCellsCallback(const sensor_msgs::PointCloud2ConstPtr& input)
    {
        // Convert sensor_msgs/PointCloud2 data to pcl/PointCloud
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*input, cloud);

        pcl::PointXYZ best_frontier;
        double best_score = -std::numeric_limits<double>::infinity();

        // Iterate frontier cells
        for (const auto& point : cloud.points)
        {
            double score = calculateCellScore(point);

            if (score > best_score)
            {
                best_frontier = point;
                best_score = score;
            }
        }
        // At this point, best_frontier contains next best cell
        ROS_INFO("Next best cell is at (%f, %f, %f) with score %f", best_frontier.x, best_frontier.y, best_frontier.z, best_score);

        // Publish best frontier
        geometry_msgs::PointStamped best_frontier_msg;
        best_frontier_msg.header.stamp = ros::Time::now();
        best_frontier_msg.header.frame_id = "uav1/world_origin";
        best_frontier_msg.point.x = best_frontier.x;
        best_frontier_msg.point.y = best_frontier.y;
        best_frontier_msg.point.z = best_frontier.z;
        best_frontier_pub.publish(best_frontier_msg);
    }

    void odomCallback(const nav_msgs::OdometryConstPtr& msg)
    {
        current_position.x = msg->pose.pose.position.x;
        current_position.y = msg->pose.pose.position.y;
        current_position.z = msg->pose.pose.position.z;
    }

    void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg)
    {
        std::unique_ptr<octomap::OcTree> octree(dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg)));

       if (octree != nullptr)
       {
           ROS_INFO("Received an Octomap message");
       }
       else
       {
           ROS_ERROR("Failed to convert Octomap message to Octree");
       }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber frontier_sub;
    ros::Subscriber octomap_sub;
    ros::Subscriber odom_sub;
    ros::Publisher best_frontier_pub;
    pcl::PointXYZ current_position;
    octomap::OcTree* octree = nullptr;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "frontier_selection");

    ROS_INFO("Starting frontier selection node");
    FrontierSelection frontier_selection;

    ros::spin();

    return 0;
}
