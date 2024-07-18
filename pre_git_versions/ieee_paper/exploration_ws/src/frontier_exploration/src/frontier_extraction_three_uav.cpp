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
#include <memory>
#include <vector>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <frontier_exploration/clusteringAlgorithm.h>

enum ExplorationState {
    OFF,
    PROXIMITY,
    SOLO,
    COOP,
    NAVIGATION,
    FAILSAFE
};

class FrontierExtraction
{
public:
    FrontierExtraction()
    {
        ros::NodeHandle nh;
        octomap_sub1 = nh.subscribe<octomap_msgs::Octomap>("uav1/octomap_server/octomap_local_full", 1, &FrontierExtraction::octomapCallback1, this);
        octomap_sub2 = nh.subscribe<octomap_msgs::Octomap>("uav2/octomap_server/octomap_local_full", 1, &FrontierExtraction::octomapCallback2, this);
        octomap_sub3 = nh.subscribe<octomap_msgs::Octomap>("uav3/octomap_server/octomap_local_full", 1, &FrontierExtraction::octomapCallback3, this);
        odom_sub1 = nh.subscribe("uav1/estimation_manager/odom_main", 1, &FrontierExtraction::odomCallback1, this);
        odom_sub2 = nh.subscribe("uav2/estimation_manager/odom_main", 1, &FrontierExtraction::odomCallback2, this);
        odom_sub3 = nh.subscribe("uav3/estimation_manager/odom_main", 1, &FrontierExtraction::odomCallback3, this);
        octomap_merged_pub = nh.advertise<octomap_msgs::Octomap>("merged_octomap", 1);
        frontier_pub = nh.advertise<visualization_msgs::Marker>("frontierCells", 1);
        cluster_pub = nh.advertise<visualization_msgs::Marker>("clusterCells", 1);
        best_frontier_pub = nh.advertise<visualization_msgs::Marker>("bestFrontier", 1);
        coverage_pub1 = nh.advertise<std_msgs::Float64MultiArray>("mappingCoverage1", 1);
        coverage_pub2 = nh.advertise<std_msgs::Float64MultiArray>("mappingCoverage2", 1);
        coverage_pub2 = nh.advertise<std_msgs::Float64MultiArray>("mappingCoverage3", 1);
        merged_coverage_pub = nh.advertise<std_msgs::Float64MultiArray>("mergedMappingCoverage", 1);
        planner_srv1 = nh.serviceClient<mrs_msgs::Vec4>("uav1/octomap_planner/goto");
        planner_srv2 = nh.serviceClient<mrs_msgs::Vec4>("uav2/octomap_planner/goto");
        planner_srv3 = nh.serviceClient<mrs_msgs::Vec4>("uav3/octomap_planner/goto");
    }

    void publishFrontierCells(const std::unique_ptr<octomap::OcTree>& octree, octomap::KeySet& cells, string ns, float r, float g, float b)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "common_origin";
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
        marker.header.frame_id = "common_origin";
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
        marker.header.frame_id = "common_origin";
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

    void octomapCallback1(const octomap_msgs::Octomap::ConstPtr& msg)
    {
        octomap::OcTree* tree1 = dynamic_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(*msg));
        tree1->expand();
        octree1.reset(tree1);

        if(mergedOctree)
            octree1 = mergeOctrees(mergedOctree, octree1);
    }

    void octomapCallback2(const octomap_msgs::Octomap::ConstPtr& msg)
    {
        octomap::OcTree* tree2 = dynamic_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(*msg));
        tree2->expand();
        octree2.reset(tree2);

        if(mergedOctree)
            octree2 = mergeOctrees(mergedOctree, octree2);
    }

    void octomapCallback3(const octomap_msgs::Octomap::ConstPtr& msg)
    {
        octomap::OcTree* tree3 = dynamic_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(*msg));
        tree3->expand();
        octree3.reset(tree3);

        if(mergedOctree)
            octree3 = mergeOctrees(mergedOctree, octree3);
    }

    void odomCallback1(const nav_msgs::OdometryConstPtr& msg)
    {
        pos1.x() = msg->pose.pose.position.x;
        pos1.y() = msg->pose.pose.position.y;
        pos1.z() = msg->pose.pose.position.z;
    }

    void odomCallback2(const nav_msgs::OdometryConstPtr& msg)
    {
        pos2.x() = msg->pose.pose.position.x;
        pos2.y() = msg->pose.pose.position.y;
        pos2.z() = msg->pose.pose.position.z;
    }

    void odomCallback3(const nav_msgs::OdometryConstPtr& msg)
    {
        pos3.x() = msg->pose.pose.position.x;
        pos3.y() = msg->pose.pose.position.y;
        pos3.z() = msg->pose.pose.position.z;
    }

    void calculateCoverage(const std::unique_ptr<octomap::OcTree>& octree, ros::Publisher publisher)
    {
        double totalCells, missingCells;
        double freeCells {0}, occupiedCells {0};
        octree->expand();

        for(octomap::OcTree::leaf_iterator it = octree->begin(), end=octree->end(); it!= end; ++it)
        {
            octomap::point3d nodePos = it.getCoordinate();
            if (nodePos.x() >= explorationMinX && nodePos.x() <= explorationMaxX && nodePos.y() >= explorationMinY && nodePos.y() <= explorationMaxY && nodePos.z() >= 0 && nodePos.z() <= explorationMaxZ)
            {
                if(!octree->isNodeOccupied(*it))
                    freeCells++;
                else if (octree->isNodeOccupied(*it))
                    occupiedCells++;
            }
        }

        totalCells = ((explorationMaxX - explorationMinX) * (explorationMaxY - explorationMinY) * explorationMaxZ) / std::pow(octree->getResolution(), 3);  // Total number of cells in the area
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
	
		publisher.publish(mappingCoverage);
    }
    
    bool proximityCheck()
    {
        double distance = (pos1 - pos2).norm();
        if (distance <= robotDistanceThreshold)
            return true;
        else
            return false;
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
        return std::sqrt(std::pow(p2.x() - p1.x(), 2) + std::pow(p2.y() - p1.y(), 2) + std::pow(p2.z() - p1.z(), 2));
    }

    double calculateInfoGain(const std::unique_ptr<octomap::OcTree>& octree, const octomap::point3d& sensorOrigin)
    {
        // Set bounding box
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
					if(!octree->search(dx, dy, dz)) unknown++;
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

    octomap::point3d frontierEvaluation(const std::unique_ptr<octomap::OcTree>& octree, octomap::KeySet& clusterCells, const octomap::point3d& pos)
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

        std::vector<double> candidateScore(candidates.size());

        for(int i = 0; i < candidates.size(); i++)
		{
			auto currCandidate = candidates[i];
			double infoGain = calculateInfoGain(octree, currCandidate);
			double distance = calculateDistance(pos, currCandidate);
			double kGain = 100.0;
			candidateScore[i] = kGain * infoGain * exp(- LAMBDA * distance); 
		}

        int maxScoreIndex = std::max_element(candidateScore.begin(), candidateScore.end()) - candidateScore.begin();
	
		octomap::point3d bestFrontier = octomap::point3d(candidates[maxScoreIndex].x(), candidates[maxScoreIndex].y(), candidates[maxScoreIndex].z());

        return bestFrontier;
    }

    std::vector<octomap::point3d> frontierEvaluation2UAV(const std::unique_ptr<octomap::OcTree>& octree, octomap::KeySet& clusterCells, const octomap::point3d& pos1, const octomap::point3d& pos2)
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

        std::vector<double> scoreVector1(candidates.size());
        std::vector<double> scoreVector2(candidates.size());

        for(int i = 0; i < candidates.size(); i++)
        {
            auto currCandidate = candidates[i];
            double infoGain = calculateInfoGain(octree, currCandidate);
            double distance1 = calculateDistance(pos1, currCandidate);
            double distance2 = calculateDistance(pos2, currCandidate);
            double kGain = 100.0;
            scoreVector1[i] = kGain * infoGain * exp(- LAMBDA * distance1); 
            scoreVector2[i] = kGain * infoGain * exp(- LAMBDA * distance2); 
        }

        int maxScoreIndex1 = std::max_element(scoreVector1.begin(), scoreVector1.end()) - scoreVector1.begin();
        octomap::point3d bestFrontier1 = octomap::point3d(candidates[maxScoreIndex1].x(), candidates[maxScoreIndex1].y(), candidates[maxScoreIndex1].z());

        // Remove the selected candidate from consideration for the second UAV
        scoreVector1.erase(scoreVector1.begin() + maxScoreIndex1);
        scoreVector2.erase(scoreVector2.begin() + maxScoreIndex1);
        candidates.erase(candidates.begin() + maxScoreIndex1);

        // Remove candidates within the threshold distance from the first best frontier
        for (int i = 0; i < candidates.size();)
        {
            if (calculateDistance(bestFrontier1, candidates[i]) < candidateDistanceThreshold)
            {
                scoreVector2.erase(scoreVector2.begin() + i);
                candidates.erase(candidates.begin() + i);
            }
            else
                ++i;
        }

        int maxScoreIndex2 = std::max_element(scoreVector2.begin(), scoreVector2.end()) - scoreVector2.begin();
        octomap::point3d bestFrontier2 = octomap::point3d(candidates[maxScoreIndex2].x(), candidates[maxScoreIndex2].y(), candidates[maxScoreIndex2].z());

        return {bestFrontier1, bestFrontier2};
    }

    void pathPlanner1(const octomap::point3d& bestFrontier)
    {
        mrs_msgs::Vec4 srv;
        srv.request.goal[0] = bestFrontier.x();
        srv.request.goal[1] = bestFrontier.y();
        srv.request.goal[2] = bestFrontier.z();
        srv.request.goal[3] = 0;

        if (planner_srv1.call(srv))
            ROS_INFO("Service called successfully");
        else
            ROS_ERROR("Failed to call service");
    }
 
    void pathPlanner2(const octomap::point3d& bestFrontier)
    {
        mrs_msgs::Vec4 srv;
        srv.request.goal[0] = bestFrontier.x();
        srv.request.goal[1] = bestFrontier.y();
        srv.request.goal[2] = bestFrontier.z();
        srv.request.goal[3] = 0;

        if (planner_srv2.call(srv))
            ROS_INFO("Service called successfully");
        else
            ROS_ERROR("Failed to call service");
    }

    std::unique_ptr<octomap::OcTree> mergeOctrees(const std::unique_ptr<octomap::OcTree>& tree1, const std::unique_ptr<octomap::OcTree>& tree2)
    {
        tree1->expand();
        tree2->expand();
        std::unique_ptr<octomap::OcTree> mergedTree = std::make_unique<octomap::OcTree>(*tree1);

        for(octomap::OcTree::leaf_iterator it = tree2->begin_leafs(), end = tree2->end_leafs(); it != end; ++it)
        {
            mergedTree->updateNode(it.getKey(), it->getValue());
        }

        return mergedTree;
    }

    void run()
	{
        ros::Rate loop_rate(10);
        octomap::KeySet frontierCells1, frontierCells2, mergedFrontierCells, clusterCells1, clusterCells2, mergedClusterCells;
        octomap::OcTreeKey currentKey1, currentKey2;
        octomap::point3d bestFrontier1, bestFrontier2;
        octomap_msgs::Octomap msg;
        std::vector<octomap::point3d> bestFrontiers;
        std::vector<octomap::point3d> neighbors;

        while (ros::ok())
        {
            ros::spinOnce();
            if(octree1) calculateCoverage(octree1, coverage_pub1);
            if(octree2) calculateCoverage(octree2, coverage_pub2);
            if(mergedOctree) calculateCoverage(mergedOctree, merged_coverage_pub);
            switch (currentState)
            {
                case OFF:
                    setState(ExplorationState::PROXIMITY);
                    break;
                case PROXIMITY:
                    if(proximityCheck())
                    {
                        mergedOctree = mergeOctrees(octree1, octree2);
                        octomap_msgs::binaryMapToMsg(*mergedOctree, msg);
                        msg.header.frame_id = "uav1/world_origin";
                        octomap_merged_pub.publish(msg);
                        setState(ExplorationState::COOP);
                    }
                    else
                    {
                        setState(ExplorationState::SOLO);
                    }
                    break;
                case SOLO:
                    frontierCells1 = frontierDetection(octree1);
                    publishFrontierCells(octree1, frontierCells1, "frontierCells1", 1.0, 0.0, 0.0);
                    frontierCells2 = frontierDetection(octree2);
                    publishFrontierCells(octree2, frontierCells2, "frontierCells2", 0.0, 1.0, 1.0);

                    clusterCells1 = frontierClustering(octree1, frontierCells1);
                    publishClusterCells(octree1, clusterCells1, "clusterCells1", 1.0, 0.5, 0.0);
                    clusterCells2 = frontierClustering(octree2, frontierCells2);
                    publishClusterCells(octree2, clusterCells2, "clusterCells2", 0.5, 0.0, 0.5);

                    bestFrontier1 = frontierEvaluation(octree1, clusterCells1, pos1);
                    publishBestFrontier(octree1, bestFrontier1, "bestFrontier1", 1.0, 1.0, 0.0);
                    bestFrontier2 = frontierEvaluation(octree2, clusterCells2, pos2);
                    publishBestFrontier(octree2, bestFrontier2, "bestFrontier2", 1.0, 0.0, 1.0);

                    setState(ExplorationState::NAVIGATION);
                    start_time = ros::Time::now();
                    break;
                case COOP:
                    mergedFrontierCells = frontierDetection(mergedOctree);
                    publishFrontierCells(mergedOctree, mergedFrontierCells, "mergedFrontierCells", 1.0, 0.0, 0.0);

                    mergedClusterCells = frontierClustering(mergedOctree, mergedFrontierCells);
                    publishClusterCells(mergedOctree, mergedClusterCells, "mergedClusterCells", 1.0, 0.5, 0.0);

                    bestFrontiers = frontierEvaluation2UAV(mergedOctree, mergedClusterCells, pos1, pos2);
                    bestFrontier1 = bestFrontiers[0];
                    publishBestFrontier(mergedOctree, bestFrontier1, "bestFrontier1", 1.0, 1.0, 0.0);
                    bestFrontier2 = bestFrontiers[1];
                    publishBestFrontier(mergedOctree, bestFrontier2, "bestFrontier2", 1.0, 0.0, 1.0);

                    setState(ExplorationState::NAVIGATION);
                    start_time = ros::Time::now();
                    break;
                case NAVIGATION:
                    pathPlanner1(bestFrontier1);
                    if(calculateDistance(pos1, bestFrontier1) < 1.0)
                    {
                        currentKey1 = octree1->coordToKey(bestFrontier1);
                        visitedCells.insert(currentKey1);

                        genNeighborCoord(octree1, currentKey1, neighbors);
                        for (const auto& neighbor : neighbors)
                        {
                            visitedCells.insert(octree1->coordToKey(neighbor));
                        }
                        neighbors.clear();
                        setState(ExplorationState::PROXIMITY);
                    }
                    pathPlanner2(bestFrontier2);
                    if(calculateDistance(pos2, bestFrontier2) < 1.0)
                    {
                        currentKey2 = octree2->coordToKey(bestFrontier2);
                        visitedCells.insert(currentKey2);

                        genNeighborCoord(octree2, currentKey2, neighbors);
                        for (const auto& neighbor : neighbors)
                        {
                            visitedCells.insert(octree2->coordToKey(neighbor));
                        }
                        neighbors.clear();
                        setState(ExplorationState::PROXIMITY);
                    }
                    if((ros::Time::now() - start_time).toSec() > timeout)
                    {
                        ROS_WARN("Timeout reached, UAV could not reach the point");
                        setState(ExplorationState::FAILSAFE);
                    }
                    if(proximityCheck() && prevState == SOLO)
                        setState(ExplorationState::PROXIMITY);
                    break;
                case FAILSAFE:
                    currentKey1 = octree1->coordToKey(bestFrontier1);
                    visitedCells.insert(currentKey1);
                    currentKey2 = octree2->coordToKey(bestFrontier2);
                    visitedCells.insert(currentKey2);

                    genNeighborCoord(octree1, currentKey1, neighbors);
                    for (const auto& neighbor : neighbors)
                    {
                        visitedCells.insert(octree1->coordToKey(neighbor));
                    }
                    neighbors.clear();
                    genNeighborCoord(octree2, currentKey2, neighbors);
                    for (const auto& neighbor : neighbors)
                    {
                        visitedCells.insert(octree2->coordToKey(neighbor));
                    }
                    neighbors.clear();
                    setState(ExplorationState::PROXIMITY);
                    break;                
                default:
                    ROS_ERROR("Invalid state");
                    return;
            }
            loop_rate.sleep();
        }
    }

    void setState(ExplorationState state)
	{
        prevState = currentState;
		currentState = state;
	}

private:
    double candidateDistanceThreshold = 5.0;
    double clusterBandwidth = 1.0;
    double explorationMinX = -30;
    double explorationMaxX = 30;
    double explorationMinY = -30;
    double explorationMaxY = 30;
    double explorationMinZ = 1.0;
    double explorationMaxZ = 8.0;
    double infoGainRange = 20.0;
    double LAMBDA = 0.13;
    double robotDistanceThreshold = 20.0;
    double timeout = 5.0;
    ExplorationState currentState = OFF;
    ExplorationState prevState;
    octomap::KeySet visitedCells;
    octomap::point3d bestFrontier;
    octomap::point3d pos1;
    octomap::point3d pos2;
    octomap::point3d pos3;
    ros::Subscriber octomap_sub1;
    ros::Subscriber octomap_sub2;
    ros::Subscriber octomap_sub3;
    ros::Subscriber odom_sub1;    
    ros::Subscriber odom_sub2;
    ros::Subscriber odom_sub3;
    ros::Publisher octomap_merged_pub;   
    ros::Publisher frontier_pub;
    ros::Publisher cluster_pub;
    ros::Publisher best_frontier_pub;
    ros::Publisher coverage_pub1;
    ros::Publisher coverage_pub2;
    ros::Publisher coverage_pub3;
    ros::Publisher merged_coverage_pub;
    ros::ServiceClient planner_srv1;
    ros::ServiceClient planner_srv2;
    ros::ServiceClient planner_srv3;
    ros::Time start_time;
    std::unique_ptr<octomap::OcTree> octree1;
    std::unique_ptr<octomap::OcTree> octree2;
    std::unique_ptr<octomap::OcTree> octree3;
    std::unique_ptr<octomap::OcTree> mergedOctree;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "frontier_extraction_three_uav");

    FrontierExtraction fem;

    fem.run();

    return 0;
}
