#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeKey.h>
#include <vector>

#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>

// #include <frontier_exploration/clustering.h>

using namespace octomap;

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

        // nh.getParam("/frontier_extraction/exploration_range", exploration_range);
        // nh.getParam("/frontier_extraction/info_gain_range", info_gain_range);
        // nh.getParam("/frontier_extraction/LAMBDA", LAMBDA);

        octomap_sub = nh.subscribe<octomap_msgs::Octomap>("uav1/octomap_server/octomap_global_full", 10, &FrontierExtraction::octomapCallback, this);
        odom_sub = nh.subscribe("uav1/estimation_manager/odom_main", 10, &FrontierExtraction::odomCallback, this);
        frontier_pub = nh.advertise<visualization_msgs::Marker>("frontierCells", 10);
        best_frontier_pub = nh.advertise<visualization_msgs::Marker>("best_frontier", 1, false);
    }

    void publishFrontierCells(const octomap::OcTree* octree, const std::vector<octomap::OcTreeKey>& cells, const std::string ns, float r, float g, float b, float a)
    {
        visualization_msgs::Marker delete_all;
        delete_all.action = visualization_msgs::Marker::DELETEALL;
        frontier_pub.publish(delete_all);

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

        frontier_pub.publish(marker);
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
        octree = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg));

        if (!octree)
        {
            ROS_ERROR("Failed to convert Octomap message to OcTree");
            return;
        }

        std::vector<octomap::OcTreeKey> frontierCells;
        frontierCells = frontierDetection(frontierCells);    

        ROS_INFO("Number of frontier cells found: %lu", frontierCells.size());

        // publishFrontier(frontierCells);

        publishFrontierCells(octree, frontierCells, "frontierCells", 1.0, 0.0, 0.0, 0.5);

        std::vector<octomap::OcTreeKey> clusteredCells;
        clusteredCells = clusterFrontier(frontierCells, clusteredCells);

        ROS_INFO("Number of frontier cells found: %lu", clusteredCells.size());

        publishFrontierCells(octree, clusteredCells, "clusteredCells", 1.0, 0.0, 1.0, 1.0);

        // double best_score = -std::numeric_limits<double>::infinity();

        // for (const octomap::OcTreeKey& key : frontierCells)
        // {
        //     octomap::point3d point = octree->keyToCoord(key);
            
        //     double score = calculateCellScore(octree, key);

        //     if (score > best_score)
        //     {
        //         bestFrontierPoint = point;
        //         best_score = score;
        //     }
        // }
        
        // publishBestFrontier();

        // std::vector<octomap::OcTreeKey> best_frontier;
        // best_frontier.push_back(octree->coordToKey(bestFrontierPoint));
        // publishMarker(octree, best_frontier, "bestFrontierPoint", 0.0, 0.0, 1.0, 1.0);

        delete octree;
    }

    void odomCallback(const nav_msgs::OdometryConstPtr& msg)
    {
        current_position.x() = msg->pose.pose.position.x;
        current_position.y() = msg->pose.pose.position.y;
        current_position.z() = msg->pose.pose.position.z;
    }

    void genNeighborCoord(octomap::OcTreeKey& start_key, std::vector<octomap::point3d>& neighbors) 
    {
        neighbors.clear();
        octomap::OcTreeKey neighbor_key;

        for (int dx = -1; dx <= 1; ++dx)
        {
            for (int dy = -1; dy <= 1; ++dy)
            {
                for (int dz = -1; dz <= 1; ++dz)
                {
                    if (dx == 0 && dy == 0 && dz == 0) continue;

                    neighbor_key = start_key;
                    neighbor_key[0] += dx;
                    neighbor_key[1] += dy;
                    neighbor_key[2] += dz;

                    octomap::point3d query = octree->keyToCoord(neighbor_key);
                    neighbors.push_back(query);
                }
            }
        }
    }

    std::vector<octomap::OcTreeKey> frontierDetection(std::vector<octomap::OcTreeKey>& frontierCells)
    {
        frontierCells.clear();
        bool unknownCellFlag{false};
        bool occupiedCellFlag{false};

        std::vector <octomap::point3d> neighbors; 

        for(octomap::OcTree::leaf_iterator it = octree->begin_leafs(), end=octree->end_leafs(); it!= end; ++it)
        {
            octomap::OcTreeKey currentKey = it.getKey();
            octomap::point3d currentPoint = octree->keyToCoord(currentKey);

            if(currentPoint.x() < -20 || currentPoint.x() > 20 || currentPoint.y() < -20 || currentPoint.y() > 20 || currentPoint.z() < 0 || currentPoint.z() > 10) continue;

            OcTreeNode* currentNode = octree->search(currentKey);
            bool isOccupied = octree->isNodeOccupied(currentNode);

            if (!isOccupied)
            {
                unknownCellFlag = false;
                occupiedCellFlag = false;

                genNeighborCoord(currentKey, neighbors);

                for (std::vector<octomap::point3d>::iterator iter = neighbors.begin(); iter != neighbors.end(); iter++)
                {
                    octomap::point3d neighborPoint =* iter;

                    OcTreeNode* neighborNode = octree-> search(neighborPoint);
                    if (neighborNode == NULL)
                        unknownCellFlag = true;
                    else if(octree->isNodeOccupied(neighborNode))
                        occupiedCellFlag = true;            
                }

                if(unknownCellFlag && !occupiedCellFlag)
                {
                    frontierCells.push_back(currentKey);
                }
            }
        }
        return frontierCells;
    }

    void keyToPointVector(std::vector<octomap::OcTreeKey>& frontierCells, std::vector<geometry_msgs::Point>& originalPointsVector)
    {
        for(std::vector<octomap::OcTreeKey>::iterator it = frontierCells.begin(), end = frontierCells.end(); it!= end; ++it)
        {
                OcTreeKey tempCell;
                tempCell = *it;

                point3d tempCellCoordinates;
                tempCellCoordinates = octree->keyToCoord(tempCell);

                geometry_msgs::Point tempCellPoint;
                tempCellPoint.x = tempCellCoordinates.x();
                tempCellPoint.y = tempCellCoordinates.y();
                tempCellPoint.z = tempCellCoordinates.z();

                originalPointsVector.push_back(tempCellPoint);
        }
    }

    void pointVectorToKey(vector<geometry_msgs::Point>& points, vector<OcTreeKey>& clusterCellsKey)
	{
		for (int i = 0; i < points.size(); i++)
		{
			point3d tempCellCoordinates;
			tempCellCoordinates.x() = points[i].x;
			tempCellCoordinates.y() = points[i].y;
			tempCellCoordinates.z() = points[i].z;
			// Transform from point to key
			OcTreeKey tempCellKey;
			if (!octree->coordToKeyChecked(tempCellCoordinates, tempCellKey)) 
			{
				OCTOMAP_ERROR_STR("Error in search: [" 
					<< tempCellCoordinates << "] is out of OcTree bounds!");
				return;
			} 
			clusterCellsKey.push_back(tempCellKey);
		}
    }

    std::vector<octomap::OcTreeKey> clusterFrontier(std::vector<octomap::OcTreeKey>& frontierCells, std::vector<octomap::OcTreeKey>& clusteredCells)
    {
        clusteredCells.clear();

        // Preprocess put the frontier cells into a vector
        std::vector<geometry_msgs::Point> originalPointsVector {};
        std::vector<geometry_msgs::Point> clusteredPointsVector {};

        keyToPointVector(frontierCells, originalPointsVector);
        MSCluster *cluster = new MSCluster();
        cluster->getMeanShiftClusters(originalPointsVector, clusteredPointsVector, 0.5);
        vector<OcTreeKey> clusterCellsKey {};
        pointVectorToKey(clusteredPointsVector, clusterCellsKey);

        for (std::vector<OcTreeKey>::iterator iter = clusterCellsKey.begin();
            iter != clusterCellsKey.end(); iter++)
            {
                clusteredCells.push_back(*iter);
            }
        
        delete cluster;
        // checkClusteredCells();
        // publishClusteredFrontier();
        return clusteredCells;
    }

    void publishFrontier(KeySet& frontierCells)
    {
        visualization_msgs::MarkerArray frontierNodesVis;
        frontierNodesVis.markers.resize(octree->getTreeDepth() +1);

        for (octomap::OcTree::iterator it = octree->begin(octree->getTreeDepth()), end = octree->end(); it != end; ++it)
        {
            bool isFrontier = false; 
            for (KeySet::iterator iter = frontierCells.begin(), end=frontierCells.end(); iter!=end; ++iter)
            {
                octomap::point3d fpoint;
                fpoint = octree->keyToCoord(*iter);
                if(it.getX() == fpoint.x() && it.getY() == fpoint.y() && it.getZ() == fpoint.z() )
                    isFrontier = true;
            }
            if (isFrontier)
            {
                double x = it.getX();
                double y = it.getY();
                double z = it.getZ();

                unsigned idx = it.getDepth();
                assert(idx < frontierNodesVis.markers.size());

                geometry_msgs:: Point cubeCenter; 
                cubeCenter.x = x; 
                cubeCenter.y = y; 
                cubeCenter.z = z; 

                frontierNodesVis.markers[idx].points.push_back(cubeCenter);
            }
        }
        for (unsigned i=0; i<frontierNodesVis.markers.size();i++)
        {

            double size = octree -> getNodeSize(i);
            frontierNodesVis.markers[i].header.frame_id = "uav1/world_origin";
            frontierNodesVis.markers[i].header.stamp = ros::Time::now();
            frontierNodesVis.markers[i].ns = "map";
            frontierNodesVis.markers[i].id = i;
            frontierNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
            frontierNodesVis.markers[i].scale.x = size;
            frontierNodesVis.markers[i].scale.y = size;
            frontierNodesVis.markers[i].scale.z = size;
            frontierNodesVis.markers[i].color.r = 1;
            frontierNodesVis.markers[i].color.g = 0;
            frontierNodesVis.markers[i].color.b = 0;
            frontierNodesVis.markers[i].color.a = 1;

            if (frontierNodesVis.markers[i].points.size() > 0)
                frontierNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
            else
                frontierNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
        }

        frontier_pub.publish(frontierNodesVis);

    }

    void publishBestFrontier()
    {
        ROS_INFO("Best frontier cell: (%f, %f, %f)", bestFrontierPoint.x(), bestFrontierPoint.y(), bestFrontierPoint.z());

        visualization_msgs::Marker frontier_goal;
        std_msgs::ColorRGBA colorgoalFrontier;
        colorgoalFrontier.r = 0;
        colorgoalFrontier.g = 1;
        colorgoalFrontier.b = 1;
        colorgoalFrontier.a = 1;
        geometry_msgs::Point cubeCenter;
        cubeCenter.x = bestFrontierPoint.x();
        cubeCenter.y = bestFrontierPoint.y();
        cubeCenter.z = bestFrontierPoint.z();
        
        frontier_goal.points.push_back(cubeCenter);
        double size = (octree->getNodeSize(octree->getTreeDepth()))*3;
        frontier_goal.header.frame_id = "uav1/world_origin";
        frontier_goal.header.stamp = ros::Time::now();
        frontier_goal.ns = "map";
        frontier_goal.type = visualization_msgs::Marker::SPHERE_LIST;
        frontier_goal.scale.x = size;
        frontier_goal.scale.y = size;
        frontier_goal.scale.z = size;
        frontier_goal.color = colorgoalFrontier;
        if (frontier_goal.points.size() > 0)
            frontier_goal.action = visualization_msgs::Marker::ADD;
        else
            frontier_goal.action = visualization_msgs::Marker::DELETE;

        best_frontier_pub.publish(frontier_goal);
    }

private:
    ros::Subscriber octomap_sub;
    ros::Subscriber odom_sub;    
    ros::Publisher frontier_pub;
    ros::Publisher best_frontier_pub;
    double exploration_range = 20.0;
    double info_gain_range = 2.0;
    double LAMBDA = 0.03;
    octomap::OcTree* octree = nullptr;
    octomap::point3d current_position;
    octomap::point3d bestFrontierPoint;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "frontier_extraction");

    FrontierExtraction frontier_extraction;

    ros::spin();

    return 0;
}
