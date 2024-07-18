#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <mrs_msgs/Vec4.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>

#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>
#include <octomap/OcTree.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <vector>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <frontier_exploration/clusteringAlgorithm.h>

enum ExplorationState {
    OFF,
    PLANNER,
    NAVIGATION,
    FAILSAFE
};

class FrontierExtraction
{
public:
    FrontierExtraction()
    {
        ros::NodeHandle nh;
        octomap_sub = nh.subscribe<octomap_msgs::Octomap>("uav1/octomap_server/octomap_local_full", 1, &FrontierExtraction::octomapCallback, this);
        odom_sub = nh.subscribe("uav1/estimation_manager/odom_main", 1, &FrontierExtraction::odomCallback, this);
        frontier_pub = nh.advertise<visualization_msgs::Marker>("frontierCells", 1);
        cluster_pub = nh.advertise<visualization_msgs::Marker>("clusterCells", 1);
        best_frontier_pub = nh.advertise<visualization_msgs::Marker>("bestFrontier", 1);
        coverage_pub = nh.advertise<std_msgs::Float64MultiArray>("mappingCoverage", 1);
        planner_srv = nh.serviceClient<mrs_msgs::Vec4>("uav1/octomap_planner/goto");
    }

    void publishFrontierCells(const std::unique_ptr<octomap::OcTree>& octree, octomap::KeySet& cells, string ns, float r, float g, float b)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "uav1/world_origin";
        marker.header.stamp = ros::Time::now();
        marker.ns = ns;
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CUBE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = octree->getResolution() / 4;
        marker.scale.y = octree->getResolution() / 4;
        marker.scale.z = octree->getResolution() / 4;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 0.5;

        for (const octomap::KeySet::value_type& key : cells)
        {
            octomap::point3d point = octree->keyToCoord(key);
            geometry_msgs::Point markerPoint;
            markerPoint.x = point.x();
            markerPoint.y = point.y();
            markerPoint.z = point.z();
            marker.points.push_back(markerPoint);
        }

        frontier_pub.publish(marker);
    }

    void publishClusterCells(const std::unique_ptr<octomap::OcTree>& octree, octomap::KeySet& clusterCells, string ns, float r, float g, float b)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "uav1/world_origin";
        marker.header.stamp = ros::Time::now();
        marker.ns = ns;
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = octree->getResolution() / 2;
        marker.scale.y = octree->getResolution() / 2;
        marker.scale.z = octree->getResolution() / 2;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 0.75;

        for (const octomap::KeySet::value_type& key : clusterCells)
        {
            octomap::point3d point = octree->keyToCoord(key);
            geometry_msgs::Point markerPoint;
            markerPoint.x = point.x();
            markerPoint.y = point.y();
            markerPoint.z = point.z();
            marker.points.push_back(markerPoint);
        }

        cluster_pub.publish(marker);
    }

    void publishBestFrontier(const std::unique_ptr<octomap::OcTree>& octree, const octomap::point3d& bestFrontierCell, string ns, float r, float g, float b)
    {
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
        marker.color.a = 1.0;

        geometry_msgs::Point markerPoint;
        markerPoint.x = bestFrontierCell.x();
        markerPoint.y = bestFrontierCell.y();
        markerPoint.z = bestFrontierCell.z();
        marker.points.push_back(markerPoint);

        best_frontier_pub.publish(marker);
    }

    void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg)
    {
        octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(*msg));
        octree->expand();
        octree1.reset(octree);
    }

    void odomCallback(const nav_msgs::OdometryConstPtr& msg)
    {
        pos.x() = msg->pose.pose.position.x;
        pos.y() = msg->pose.pose.position.y;
        pos.z() = msg->pose.pose.position.z;
    }

    void calculateCoverage(const std::unique_ptr<octomap::OcTree>& octree)
    {
        double totalCells, missingCells;
        double freeCells, occupiedCells {0};

        for(octomap::OcTree::leaf_iterator it = octree->begin(), end=octree->end(); it!= end; ++it)
        {
            octomap::point3d nodePos = it.getCoordinate();
            if (nodePos.x() >= explorationMinX && nodePos.x() <= explorationMaxX && nodePos.y() >= explorationMinY && nodePos.y() <= explorationMaxY && nodePos.z() >= explorationMinZ && nodePos.z() <= explorationMaxZ)
            {
                if(!octree->isNodeOccupied(*it))
                    freeCells++;
                else if (octree->isNodeOccupied(*it))
                    occupiedCells++;
            }
        }

        totalCells = ((explorationMaxX - explorationMinX) * (explorationMaxY - explorationMinY) * (explorationMaxZ - explorationMinZ)) / std::pow(octree->getResolution(), 3);  // Total number of cells in the area
        missingCells = totalCells - freeCells - occupiedCells;

        double percentageMapped = 100.0 * (freeCells + occupiedCells) / totalCells;

        ROS_INFO("Mapped: %f", percentageMapped);

        std_msgs::Float64MultiArray mappingCoverage;
		mappingCoverage.data.resize(5);
		mappingCoverage.data[0] = ros::Time::now().toSec();
		mappingCoverage.data[1] = occupiedCells / totalCells;
		mappingCoverage.data[2] = freeCells / totalCells;
		mappingCoverage.data[3] = missingCells / totalCells;
		mappingCoverage.data[4] = totalCells / totalCells;
	
		coverage_pub.publish(mappingCoverage);
    }

    void keyToPointVector(const std::unique_ptr<octomap::OcTree>& octree, octomap::KeySet& frontierCells, std::vector<geometry_msgs::Point>& originalPointsVector)
    {
		for(octomap::KeySet::iterator iter = frontierCells.begin(), end = frontierCells.end(); iter!= end; ++iter)
		{
            octomap::OcTreeKey tempCell;
            tempCell = *iter;

            octomap::point3d tempCellCoordinates;
            tempCellCoordinates = octree->keyToCoord(tempCell);

            geometry_msgs::Point tempCellPoint;
            tempCellPoint.x = tempCellCoordinates.x();
            tempCellPoint.y = tempCellCoordinates.y();
            tempCellPoint.z = tempCellCoordinates.z();

            originalPointsVector.push_back(tempCellPoint);
		}
	}

    void pointVectorToKey(const std::unique_ptr<octomap::OcTree>& octree, std::vector<geometry_msgs::Point>& points, std::vector<octomap::OcTreeKey>& clusterCellsKey)
	{
		for (int i = 0; i < points.size(); i++)
		{
			octomap::point3d tempCellCoordinates;
			tempCellCoordinates.x() = points[i].x;
			tempCellCoordinates.y() = points[i].y;
			tempCellCoordinates.z() = points[i].z;
			// Transform from point to key
			octomap::OcTreeKey tempCellKey;
			if (!octree->coordToKeyChecked(tempCellCoordinates, tempCellKey)) 
			{
				OCTOMAP_ERROR_STR("Error in search: [" 
					<< tempCellCoordinates << "] is out of OcTree bounds!");
				return;
			} 
			clusterCellsKey.push_back(tempCellKey);
		}
	}

    void genNeighborCoord(const std::unique_ptr<octomap::OcTree>& octree, const octomap::OcTreeKey& start_key, std::vector<octomap::point3d>& neighbors) 
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

    double calculateDistance(const octomap::point3d& p1, const octomap::point3d& p2)
    {		
        return (p1 - p2).norm();
    }

    double calculateInfoGain(const std::unique_ptr<octomap::OcTree>& octree, const octomap::point3d& sensorOrigin)
    {
		octomap::point3d minPoint, maxPoint;

        double resolution = octree->getResolution();

        minPoint.x() = std::max(sensorOrigin.x() - (infoGainRange / 2), explorationMinX);
        minPoint.y() = std::max(sensorOrigin.y() - (infoGainRange / 2), explorationMinY);
        minPoint.z() = std::max(sensorOrigin.z() - (infoGainRange / 2), explorationMinZ);

        maxPoint.x() = std::min(sensorOrigin.x() + (infoGainRange / 2), explorationMaxX);
        maxPoint.y() = std::min(sensorOrigin.y() + (infoGainRange / 2), explorationMaxY);
        maxPoint.z() = std::min(sensorOrigin.z() + (infoGainRange / 2), explorationMaxZ);

		int unknown {0};
		int total {0};

		for(double dx = minPoint.x(); dx < maxPoint.x(); dx += resolution)
		{
			for(double dy = minPoint.y(); dy < maxPoint.y(); dy += resolution)
			{
				for (double dz = minPoint.z(); dz < maxPoint.z(); dz += resolution)
				{
					total++;
					if(!octree->search(dx, dy, dz))
						unknown++;
				}
			}
		}	
		return (double)unknown / (double)total;
    }

    octomap::KeySet frontierDetection(const std::unique_ptr<octomap::OcTree>& octree)
    {
        octomap::KeySet frontierCells; 

        bool unknownCellFlag {false};
        bool occupiedCellFlag {false};

        std::vector <octomap::point3d> neighbors; 

        for(octomap::OcTree::leaf_iterator it = octree->begin_leafs(), end=octree->end_leafs(); it!= end; ++it)
        {
            octomap::OcTreeKey currentKey = it.getKey();
            octomap::point3d currentPoint = octree->keyToCoord(currentKey);

            if(currentPoint.x() < explorationMinX || currentPoint.x() > explorationMaxX || currentPoint.y() < explorationMinY || currentPoint.y() > explorationMaxY || currentPoint.z() < explorationMinZ || currentPoint.z() > explorationMaxZ) continue;

            octomap::OcTreeNode* currentNode = octree->search(currentKey);
            bool isOccupied = octree->isNodeOccupied(currentNode);

            if (!isOccupied && visitedCells.find(currentKey) == visitedCells.end())
            {
                unknownCellFlag = false;
                occupiedCellFlag = false;

                genNeighborCoord(octree, currentKey, neighbors);

                for (std::vector<octomap::point3d>::iterator iter = neighbors.begin(); iter != neighbors.end(); iter++)
                {
                    octomap::point3d neighborPoint =* iter;

                    octomap::OcTreeNode* neighborNode = octree-> search(neighborPoint);
                    if (neighborNode == NULL)
                        unknownCellFlag = true;
                    else if(octree->isNodeOccupied(neighborNode))
                        occupiedCellFlag = true;            
                }

                if(unknownCellFlag && !occupiedCellFlag)
                {
                    frontierCells.insert(currentKey);
                }
            }
        }
        return frontierCells;
    }

    octomap::KeySet frontierClustering(const std::unique_ptr<octomap::OcTree>& octree, octomap::KeySet& frontierCells)
    {
		octomap::KeySet clusterCells;
		
		// Preprocess put the frontier cells into a vector
		std::vector<geometry_msgs::Point> originalPointsVector {};
		std::vector<geometry_msgs::Point> clusteredPointsVector {};

		keyToPointVector(octree, frontierCells, originalPointsVector);
		MSCluster *cluster = new MSCluster();
		cluster->getMeanShiftClusters(originalPointsVector, clusteredPointsVector, clusterBandwidth);
		std::vector<octomap::OcTreeKey> clusterCellsKey {};
		pointVectorToKey(octree, clusteredPointsVector, clusterCellsKey);
		for (std::vector<octomap::OcTreeKey>::iterator iter = clusterCellsKey.begin(); iter != clusterCellsKey.end(); iter++)
		{
			clusterCells.insert(*iter);
		}
		delete cluster;

        return clusterCells;
	}

    octomap::point3d frontierEvaluation(const std::unique_ptr<octomap::OcTree>& octree, octomap::KeySet& clusterCells)
    {
        std::vector<octomap::point3d> candidates;

        for(octomap::KeySet::iterator iter = clusterCells.begin(), end = clusterCells.end(); iter != end; ++iter)
		{
			// Get cell position
			octomap::point3d tempCellPosition = octree->keyToCoord(*iter);
			double x = tempCellPosition.x();
			double y = tempCellPosition.y();
			double z = tempCellPosition.z();
			
			candidates.push_back(octomap::point3d(x, y, z));	
		}

        std::vector<double> scoreVector(candidates.size());

        for(int i = 0; i < candidates.size(); i++)
		{
			// Get candidate
			auto currCandidate = candidates[i];
			double infoGain = calculateInfoGain(octree, currCandidate);
			double distance = calculateDistance(pos, currCandidate);
			double kGain = 100.0;
			scoreVector[i] = kGain * infoGain * exp(- LAMBDA * distance); 
		}

        int maxScoreIndex = std::max_element(scoreVector.begin(), scoreVector.end()) - scoreVector.begin();
	
		// Define best frontier
		octomap::point3d bestFrontier = octomap::point3d(candidates[maxScoreIndex].x(), candidates[maxScoreIndex].y(), candidates[maxScoreIndex].z());

        return bestFrontier;
    }

    void pathPlanner(const octomap::point3d& bestFrontier)
    {
        mrs_msgs::Vec4 srv;
        srv.request.goal[0] = bestFrontier.x();
        srv.request.goal[1] = bestFrontier.y();
        srv.request.goal[2] = bestFrontier.z();
        srv.request.goal[3] = 0;

        if (planner_srv.call(srv))
            ROS_INFO("Service called successfully");
        else
            ROS_ERROR("Failed to call service");
    }

    void run()
	{
        ros::Rate loop_rate(10);
        octomap::KeySet frontierCells, clusterCells;
        octomap::point3d bestFrontier;
        octomap::OcTreeKey currentKey;
        std::vector<octomap::point3d> neighbors;

        while (ros::ok())
        {
            ros::spinOnce();
            if(octree1)
            {
                switch (currentState)
                {
                    case OFF:
                        setState(ExplorationState::PLANNER);
                        break;
                    case PLANNER:
                        if(octree1) calculateCoverage(octree1);

                        frontierCells = frontierDetection(octree1);
                        publishFrontierCells(octree1, frontierCells, "frontierCells", 1.0, 0.0, 0.0);
                        clusterCells = frontierClustering(octree1, frontierCells);
                        publishClusterCells(octree1, clusterCells, "clusterCells", 1.0, 0.5, 0.0);
                        bestFrontier = frontierEvaluation(octree1, clusterCells);
                        publishBestFrontier(octree1, bestFrontier, "bestFrontier", 1.0, 1.0, 0.0);
                        
                        setState(ExplorationState::NAVIGATION);
                        start_time = ros::Time::now();
                        break;
                    case NAVIGATION:
                        pathPlanner(bestFrontier);
                        ros::Duration(0.05).sleep();
                        if(calculateDistance(pos, bestFrontier) < 1.0)
                        {
                            currentKey = octree1->coordToKey(bestFrontier);
                            visitedCells.insert(currentKey);

                            genNeighborCoord(octree1, currentKey, neighbors);
                            for (const auto& neighbor : neighbors)
                            {
                                visitedCells.insert(octree1->coordToKey(neighbor));
                            }
                            setState(ExplorationState::PLANNER);
                        }
                        else if ((ros::Time::now() - start_time).toSec() > timeout)
                        {
                            ROS_WARN("Timeout reached, UAV could not reach the point");
                            setState(ExplorationState::FAILSAFE);
                        }
                        break;
                    case FAILSAFE:
                        currentKey = octree1->coordToKey(bestFrontier);
                        visitedCells.insert(currentKey);

                        genNeighborCoord(octree1, currentKey, neighbors);
                        for (const auto& neighbor : neighbors)
                        {
                            visitedCells.insert(octree1->coordToKey(neighbor));
                        }

                        setState(ExplorationState::PLANNER);
                        break;
                    default:
                        ROS_ERROR("Invalid state");
                        return;
                }
            }
            loop_rate.sleep();
        }
    }

    void setState(ExplorationState state)
	{
		currentState = state;
	}

private:
    double clusterBandwidth = 1.0;
    double explorationMinX = -30;
    double explorationMaxX = 30;
    double explorationMinY = -30;
    double explorationMaxY = 30;
    double explorationMinZ = 1.0;
    double explorationMaxZ = 8.0;
    double infoGainRange = 20.0;
    double LAMBDA = 0.1;
    double timeout = 5.0;
    ExplorationState currentState = OFF;
    octomap::point3d pos;
    octomap::KeySet visitedCells;
    ros::Subscriber octomap_sub;
    ros::Subscriber odom_sub;    
    ros::Publisher frontier_pub;
    ros::Publisher cluster_pub;
    ros::Publisher best_frontier_pub;
    ros::Publisher coverage_pub;
    ros::ServiceClient planner_srv;
    ros::Time start_time;
    std::unique_ptr<octomap::OcTree> octree1;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "frontier_extraction");

    FrontierExtraction fe;

    fe.run();

    return 0;
}
