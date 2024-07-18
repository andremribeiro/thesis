#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <visualization_msgs/Marker.h>

struct OcTreeKeyHash
{
    std::size_t operator()(const octomap::OcTreeKey& key) const
    {
        return std::hash<unsigned short>()(key[0]) ^
               (std::hash<unsigned short>()(key[1]) << 1) ^
               (std::hash<unsigned short>()(key[2]) << 2);
    }
};

class FrontierExtraction
{
public:
    FrontierExtraction()
    {
        ros::NodeHandle nh;
        nh.getParam("/frontier_extraction/info_gain_range", info_gain_range);
        nh.getParam("/frontier_extraction/LAMBDA", LAMBDA);
        octomap_sub = nh.subscribe<octomap_msgs::Octomap>("octomap_in", 10, &FrontierExtraction::octomapCallback, this);
        odom_sub = nh.subscribe("odom_in", 10, &FrontierExtraction::odomCallback, this);
        marker_pub = nh.advertise<visualization_msgs::Marker>("frontier_cells", 10);
        best_frontier_pub = nh.advertise<geometry_msgs::PointStamped>("best_frontier", 10);
    }

    void publishMarker(const octomap::OcTree* octree, const std::vector<octomap::OcTreeKey>& cells, const std::string ns, float r, float g, float b, float a)
    {
        visualization_msgs::Marker delete_all;
        delete_all.action = visualization_msgs::Marker::DELETEALL;
        marker_pub.publish(delete_all);

        visualization_msgs::Marker marker;
        marker.header.frame_id = "uav1/world_origin";
        marker.header.stamp = ros::Time::now();
        marker.ns = ns;
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CUBE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = octree->getResolution() / 2;
        marker.scale.y = octree->getResolution() / 2;
        marker.scale.z = octree->getResolution() / 2;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = a;

        for (const octomap::OcTreeKey& key : cells)
        {
            octomap::point3d point = octree->keyToCoord(key);
            geometry_msgs::Point marker_point;
            marker_point.x = point.x();
            marker_point.y = point.y();
            marker_point.z = point.z();
            marker.points.push_back(marker_point);
        }

        marker_pub.publish(marker);
    }

    std::pair<std::vector<octomap::OcTreeKey>, std::vector<octomap::OcTreeKey>> getNeighbors(const octomap::OcTree* octree, const octomap::OcTreeKey& key, float range)
    {
        int max_neighbors = std::pow((2 * range + 1), 3);
        std::vector<octomap::OcTreeKey> occupied_neighbors;
        std::vector<octomap::OcTreeKey> unknown_neighbors;
        occupied_neighbors.reserve(max_neighbors);
        unknown_neighbors.reserve(max_neighbors);

        octomap::OcTreeKey neighbor_key;

        for (int dx = -range; dx <= range; ++dx)
        {
            for (int dy = -range; dy <= range; ++dy)
            {
                for (int dz = -range; dz <= range; ++dz)
                {
                    if (dx == 0 && dy == 0 && dz == 0)
                        continue;

                    neighbor_key = key;
                    neighbor_key[0] += dx;
                    neighbor_key[1] += dy;
                    neighbor_key[2] += dz;

                    octomap::OcTreeNode* node = octree->search(neighbor_key);
                    if (node == NULL)
                    {
                        unknown_neighbors.push_back(neighbor_key);
                    }
                    else if (octree->isNodeOccupied(node))
                    {
                        occupied_neighbors.push_back(neighbor_key);
                    }
                }
            }
        }

        return std::make_pair(occupied_neighbors, unknown_neighbors);
    }

    double calculateInfoGain(const octomap::OcTree* octree, const octomap::OcTreeKey& key)
    {
        std::vector<octomap::OcTreeKey> unknown_neighbors = getNeighbors(octree, key, info_gain_range).second;

        double total_neighbors = std::pow(2 * info_gain_range + 1, 3) - 1;

        double info_gain = 0.0;
        if (total_neighbors > 0)
        {
            info_gain = unknown_neighbors.size() / total_neighbors;
        }

        return info_gain;
    }
    
    double calculateCellScore(const octomap::OcTree* octree, const octomap::OcTreeKey& key)
    {
        octomap::point3d cell_position = octree->keyToCoord(key);
        
        double info_gain = calculateInfoGain(octree, key);

        double distance = (cell_position - current_position).norm();

        return info_gain / std::exp(LAMBDA * distance);
    }

    void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg)
    {
        octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg));

        if (!octree)
        {
            ROS_ERROR("Failed to convert Octomap message to OcTree");
            return;
        }

        std::vector<octomap::OcTreeKey> frontier_cells;
        std::vector<octomap::OcTreeKey> safe_frontier_cells;

        frontier_cells.clear();
        safe_frontier_cells.clear();

        for(octomap::OcTree::leaf_iterator it = octree->begin_leafs(), end=octree->end_leafs(); it!= end; ++it)
        {
            if(octree->isNodeOccupied(*it)) continue;

            std::pair<std::vector<octomap::OcTreeKey>, std::vector<octomap::OcTreeKey>> neighbors = getNeighbors(octree, it.getKey(), 2);

            std::vector<octomap::OcTreeKey> occupied_neighbors = neighbors.first;
            std::vector<octomap::OcTreeKey> unknown_neighbors = neighbors.second;
            octomap::point3d cell_position = octree->keyToCoord(it.getKey());

            if (cell_position.x() >= -20 && cell_position.x() <= 20 && cell_position.y() >= -20 && cell_position.y() <= 20)
            {
                if (!unknown_neighbors.empty() && cell_position.z() >= 0.5)
                {
                    frontier_cells.push_back(it.getKey());
                    if (occupied_neighbors.empty())
                    {
                        safe_frontier_cells.push_back(it.getKey());
                    }
                }  
            }   
        }

        ROS_INFO("Number of frontier cells found: %lu", frontier_cells.size());
        ROS_INFO("Number of safe frontier cells found: %lu", safe_frontier_cells.size());

        publishMarker(octree, frontier_cells, "frontier_cells", 1.0, 0.0, 0.0, 0.25);
        publishMarker(octree, safe_frontier_cells, "safe_frontier_cells", 1.0, 0.0, 1.0, 0.5);

        octomap::point3d best_frontier_cell;
        double best_score = -std::numeric_limits<double>::infinity();

        for (const octomap::OcTreeKey& key : safe_frontier_cells)
        {
            octomap::point3d point = octree->keyToCoord(key);
            
            double score = calculateCellScore(octree, key);

            if (score > best_score)
            {
                best_frontier_cell = point;
                best_score = score;
            }
        }
        
        ROS_INFO("Best frontier cell: (%f, %f, %f), score: %f", best_frontier_cell.x(), best_frontier_cell.y(), best_frontier_cell.z(), best_score);

        // std::vector<octomap::OcTreeKey> best_frontier;
        // best_frontier.push_back(octree->coordToKey(best_frontier_cell));
        // publishMarker(octree, best_frontier, "best_frontier_cell", 0.0, 0.0, 1.0, 1.0);

        geometry_msgs::PointStamped best_frontier_msg;
        best_frontier_msg.header.stamp = ros::Time::now();
        best_frontier_msg.header.frame_id = "uav1/world_origin";
        best_frontier_msg.point.x = best_frontier_cell.x();
        best_frontier_msg.point.y = best_frontier_cell.y();
        best_frontier_msg.point.z = best_frontier_cell.z();
        best_frontier_pub.publish(best_frontier_msg);

        ros::Duration(4.0).sleep();

        delete octree;
    }

    void odomCallback(const nav_msgs::OdometryConstPtr& msg)
    {
        current_position.x() = msg->pose.pose.position.x;
        current_position.y() = msg->pose.pose.position.y;
        current_position.z() = msg->pose.pose.position.z;
    }

private:
    ros::Subscriber octomap_sub;
    ros::Subscriber odom_sub;    
    ros::Publisher marker_pub;
    ros::Publisher best_frontier_pub;
    double info_gain_range;
    double LAMBDA;
    octomap::point3d current_position;    
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "frontier_extraction");

    FrontierExtraction frontier_extraction;

    ros::spin();

    return 0;
}
