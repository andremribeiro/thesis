#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <mrs_msgs/Vec4.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Trigger.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>

#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>
#include <octomap/OcTree.h>

#include <cmath>
#include <functional>
#include <memory>
#include <set>
#include <unordered_map>
#include <vector>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <frontier_exploration/clusteringAlgorithm.h>

class UAV
{
public:
    UAV(ros::NodeHandle& nh, ros::NodeHandle& nh_private, const std::string& uav_name, int i)
        : uav_index(i)
    {
        octomap_sub = nh.subscribe<octomap_msgs::Octomap>(uav_name + "/octomap_binary", 1, std::bind(&UAV::octomapCallback, this, std::placeholders::_1));
        planner_sub = nh.subscribe<std_msgs::Bool>(uav_name + "/octomap_planner/path_available", 1, std::bind(&UAV::plannerCallback, this, std::placeholders::_1));
        odom_sub = nh.subscribe<nav_msgs::Odometry>(uav_name + "/estimation_manager/odom_main", 1, std::bind(&UAV::odomCallback, this, std::placeholders::_1));
        octomap_pub = nh.advertise<octomap_msgs::Octomap>(uav_name + "/shared_octomap", 1);
        frontier_pub = nh.advertise<visualization_msgs::Marker>(uav_name + "/frontier_markers", 1);
        frontier_parent_pub = nh.advertise<visualization_msgs::Marker>(uav_name + "/frontier_parent_markers", 1);
        cluster_pub = nh.advertise<visualization_msgs::Marker>(uav_name + "/cluster_markers", 1);
        waypoint_pub = nh.advertise<visualization_msgs::Marker>(uav_name + "/waypoint_marker", 1);
        coverage_pub = nh.advertise<std_msgs::Float64MultiArray>(uav_name + "/coverage", 1);
        uavs_in_range_pub = nh.advertise<std_msgs::Float64MultiArray>(uav_name + "/uavs_in_range", 1);
        hover_srv = nh.serviceClient<std_srvs::Trigger>(uav_name + "/octomap_planner/stop");
        planner_srv = nh.serviceClient<mrs_msgs::Vec4>(uav_name + "/octomap_planner/goto");

        nh_private.param("bandwidth", bandwidth, 1.0);
        nh_private.param("lambda", lambda, 0.14);
        nh_private.param("r_exp", r_exp, 1.0);
        nh_private.param("sensor_range", sensor_range, 40.0);
        nh_private.param("waypoint_distance_threshold", waypoint_distance_threshold, 15.0);
        nh_private.param("exploration_min_x", exploration_min_x, -30.0);
        nh_private.param("exploration_max_x", exploration_max_x, 30.0);
        nh_private.param("exploration_min_y", exploration_min_y, -30.0);
        nh_private.param("exploration_max_y", exploration_max_y, 30.0);
        nh_private.param("exploration_min_z", exploration_min_z, 10.0);
        nh_private.param("exploration_max_z", exploration_max_z, 0.0);
    }

    void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg)
    {
        octomap::OcTree* temp = dynamic_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(*msg));

        if(temp)
        {
            temp->expand();
            if(octree)
            {
                for(auto it = temp->begin_leafs(), end = temp->end_leafs(); it != end; ++it)
                {
                    octomap::point3d node_pos = it.getCoordinate();
                    double distance = sqrt(pow(node_pos.x() - pos.x(), 2) + pow(node_pos.y() - pos.y(), 2) + pow(node_pos.z() - pos.z(), 2));
                    
                    if(distance <= 0.3)
                        octree->updateNode(it.getKey(), false); // Set node as free
                    else
                        octree->updateNode(it.getKey(), it->getValue());
                }
                delete temp;

                octomap_msgs::Octomap msg_out;
                octomap_msgs::binaryMapToMsg(*octree, msg_out);
                msg_out.header.frame_id = "common_origin";
                octomap_pub.publish(msg_out);
            }
            else
                octree.reset(temp);
        }
        else 
            ROS_ERROR("Failed to convert octomap message to OcTree for UAV %d", uav_index);

        coverage();
    }

    void plannerCallback(const std_msgs::Bool::ConstPtr& msg)
    {     
        path_available = msg->data;
    }
    
    void odomCallback(const nav_msgs::OdometryConstPtr& msg)
    {     
        pos.x() = msg->pose.pose.position.x;
        pos.y() = msg->pose.pose.position.y;
        pos.z() = msg->pose.pose.position.z;        
    }

    void frontierDetection()
    {
        frontiers.clear();

        bool unknown = false;
        bool occupied = false;

        std::vector <octomap::point3d> neighbors;

        for(octomap::OcTree::leaf_iterator it = octree->begin_leafs(), end=octree->end_leafs(); it!= end; ++it)
        {
            octomap::OcTreeKey current_key = it.getKey();
            octomap::point3d current_point = octree->keyToCoord(current_key);

            if (!isWithinExplorationBounds(current_point))
                continue;

            octomap::OcTreeNode* current_node = octree->search(current_key);
            bool isOccupied = octree->isNodeOccupied(current_node);

            if (!isOccupied)
            {
                unknown = false;
                occupied = false;

                getNeighbor(current_key, neighbors);

                for (std::vector<octomap::point3d>::iterator iter = neighbors.begin(); iter != neighbors.end(); iter++)
                {
                    octomap::point3d ngbr_point =* iter;

                    if (!isWithinExplorationBounds(ngbr_point))
                        continue;

                    octomap::OcTreeNode* ngbr_node = octree-> search(ngbr_point);
                    if (ngbr_node == NULL)
                        unknown = true;
                    else if(octree->isNodeOccupied(ngbr_node))
                        occupied = true;            
                }

                if(unknown && !occupied)
                    frontiers.insert(current_key);
            }
        }
        ROS_INFO("UAV %d found %zu frontiers", uav_index, frontiers.size());

        publishMarker(frontiers, "frontiers", 1, frontier_pub);
    }

    void searchParents()
    {
        frontier_parents.clear();
        int depth = octree->getTreeDepth() - std::log2(r_exp / octree->getResolution());
        octomap::KeySet frontier_children;
        
        for(octomap::KeySet::iterator iter = frontiers.begin(), end = frontiers.end(); iter != end; ++iter)
        {
            if(octree->search(*iter, depth))
                frontier_children.insert(*iter);
        }

        for(octomap::OcTree::iterator it = octree->begin(depth), end = octree->end(); it != end; ++it)
        {
            if(frontier_children.find(it.getKey()) != frontier_children.end())
                frontier_parents.insert(it.getKey());
        }
        ROS_INFO("UAV %d found %zu frontier parents", uav_index, frontier_parents.size());

        publishMarker(frontier_parents, "frontier_parents", 0.50, frontier_parent_pub);
    }

    bool isWithinExplorationBounds(const octomap::point3d& point)
    {
        return (point.x() >= exploration_min_x && point.x() <= exploration_max_x &&
                point.y() >= exploration_min_y && point.y() <= exploration_max_y &&
                point.z() >= exploration_min_z && point.z() <= exploration_max_z);
    }

    void getNeighbor(const octomap::OcTreeKey& start_key, std::vector<octomap::point3d>& neighbors) 
    {
        neighbors.clear();
        octomap::OcTreeKey neighbor_key;

        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    if (dx == 0 && dy == 0 && dz == 0)
                        continue;

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

    void frontierClustering()
    {
		clusters.clear();
		
		std::vector<geometry_msgs::Point> original_points {};
		std::vector<geometry_msgs::Point> clustered_points {};

		keyToPointVector(frontier_parents, original_points);
		MSCluster *cluster = new MSCluster();
		cluster->getMeanShiftClusters(original_points, clustered_points, bandwidth);
		std::vector<octomap::OcTreeKey> clusters_key {};
		pointVectorToKey(clustered_points, clusters_key);

		for (std::vector<octomap::OcTreeKey>::iterator iter = clusters_key.begin(); iter != clusters_key.end(); iter++)
			clusters.insert(*iter);

		delete cluster;

        ROS_INFO("UAV %d found %zu clusters", uav_index, clusters.size());

        publishMarker(clusters, "clusters", 0.50, cluster_pub);
	}

    void keyToPointVector(octomap::KeySet& frontierCells, std::vector<geometry_msgs::Point>& original_points)
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

            original_points.push_back(tempCellPoint);
		}
	}

    void pointVectorToKey(std::vector<geometry_msgs::Point>& points, std::vector<octomap::OcTreeKey>& clusters_key)
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
			clusters_key.push_back(tempCellKey);
		}
	}

    octomap::point3d frontierEvaluation(const std::vector<octomap::point3d>& assigned_waypoints)
    {
        candidates.clear();

        for(octomap::KeySet::iterator iter = clusters.begin(), end = clusters.end(); iter != end; ++iter)
        {
            octomap::point3d tempCellPosition = octree->keyToCoord(*iter);
            double x = tempCellPosition.x();
            double y = tempCellPosition.y();
            double z = tempCellPosition.z();
            octomap::point3d candidate = octomap::point3d(x, y, z);

            bool add = true;
            for (const auto& assigned_waypoint : assigned_waypoints)
            {
                if(calculateDistance(candidate, assigned_waypoint) < waypoint_distance_threshold)
                {
                    add = false;
                    break;
                }
            }
            if(add)
                candidates.push_back(candidate);
        }

        std::vector<double> candidate_scores(candidates.size());

        for(int i = 0; i < candidates.size(); i++)
        {
            auto current_candidate = candidates[i];
            double distance = calculateDistance(pos, current_candidate);
            double info_gain = calculateInfoGain(current_candidate);
            candidate_scores[i] = 100.0 * info_gain * exp(-lambda * distance);

            auto current_key = octree->coordToKey(current_candidate);
            if(unreached.find(current_key) != unreached.end())
            {
                int count = unreached[current_key];
                candidate_scores[i] *= pow(0.5, count);
            }
        }

        int max_score_index = std::max_element(candidate_scores.begin(), candidate_scores.end()) - candidate_scores.begin();

        ROS_INFO("Waypoint (%.2f,%.2f,%.2f) for UAV %d selected with score: %f", candidates[max_score_index].x(), candidates[max_score_index].y(), candidates[max_score_index].z(), uav_index, candidate_scores[max_score_index]);

        waypoint_score = candidate_scores[max_score_index];

        waypoint = candidates[max_score_index];

        return waypoint;
    }

    double calculateDistance(const octomap::point3d& p1, const octomap::point3d& p2)
    {		
        return std::sqrt(std::pow(p2.x() - p1.x(), 2) + std::pow(p2.y() - p1.y(), 2) + std::pow(p2.z() - p1.z(), 2));
    }

    double calculateInfoGain(const octomap::point3d& sensor_origin)
    {
        octomap::point3d bbx_min, bbx_max;

        bbx_min.x() = std::max(sensor_origin.x() - (sensor_range), exploration_min_x);
        bbx_min.y() = std::max(sensor_origin.y() - (sensor_range), exploration_min_y);
        bbx_min.z() = std::max(sensor_origin.z() - (sensor_range), exploration_min_z);

        bbx_max.x() = std::min(sensor_origin.x() + (sensor_range), exploration_max_x);
        bbx_max.y() = std::min(sensor_origin.y() + (sensor_range), exploration_max_y);
        bbx_max.z() = std::min(sensor_origin.z() + (sensor_range), exploration_max_z);

        int unknown = 0;
        int total = 0;

        for(double dx = bbx_min.x(); dx < bbx_max.x(); dx += r_exp) {
            for(double dy = bbx_min.y(); dy < bbx_max.y(); dy += r_exp) {
                for (double dz = bbx_min.z(); dz < bbx_max.z(); dz += r_exp) {
                    octomap::point3d direction(dx - sensor_origin.x(), dy - sensor_origin.y(), dz - sensor_origin.z());
                    double elevation_angle = atan2(direction.z(), sqrt(direction.x() * direction.x() + direction.y() * direction.y()));

                    // Check if the point is within the sensor's vertical FOV
                    if(abs(elevation_angle) <= M_PI / 4) { // 90 degrees FOV
                        total++;
                        if(!octree->search(dx, dy, dz)) unknown++;
                    }
                }
            }
        }   
        return (double)unknown / (double)total;
    }

    void publishMarker(octomap::KeySet& data, const std::string& ns, double scale, ros::Publisher& pub)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "common_origin";
        marker.header.stamp = ros::Time::now();
        marker.ns = ns;
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = scale * octree->getResolution();
        marker.scale.y = scale * octree->getResolution();
        marker.scale.z = scale * octree->getResolution();

        if (ns == "frontiers") {
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 0.5;
        } else if (ns == "frontier_parents") {
            marker.color.r = 1.0;
            marker.color.g = 0.5;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
        } else if (ns == "clusters") {
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
        }

        for (const octomap::KeySet::value_type& key : data)
        {
            octomap::point3d point = octree->keyToCoord(key);
            geometry_msgs::Point markerPoint;
            markerPoint.x = point.x();
            markerPoint.y = point.y();
            markerPoint.z = point.z();
            marker.points.push_back(markerPoint);
        }
        pub.publish(marker);
    }

    void publishWaypoint()
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "common_origin";
        marker.header.stamp = ros::Time::now();
        marker.ns = "waypoint";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = octree->getResolution();
        marker.scale.y = octree->getResolution();
        marker.scale.z = octree->getResolution();
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        geometry_msgs::Point markerPoint;
        markerPoint.x = waypoint.x();
        markerPoint.y = waypoint.y();
        markerPoint.z = waypoint.z();
        marker.points.push_back(markerPoint);

        waypoint_pub.publish(marker);
    }

    void coverage()
    {
        double total_cells, missing_cells;
        double free_cells {0}, occupied_cells {0};

        octree->expand();

        for(octomap::OcTree::leaf_iterator it = octree->begin(), end=octree->end(); it!= end; ++it)
        {
            octomap::point3d nodePos = it.getCoordinate();
            if(isWithinExplorationBounds(nodePos))
            {
                if(!octree->isNodeOccupied(*it))
                    free_cells++;
                else if (octree->isNodeOccupied(*it))
                    occupied_cells++;
            }
        }

        total_cells = ((exploration_max_x - exploration_min_x) * (exploration_max_y - exploration_min_y) * (exploration_max_z - exploration_min_z)) / std::pow(octree->getResolution(), 3);
        missing_cells = total_cells - free_cells - occupied_cells;

        double mapped = 100.0 * (free_cells + occupied_cells) / total_cells;

        ROS_INFO("UAV %d mapped: %.2f %%", uav_index, mapped);

        std_msgs::Float64MultiArray coverage;
		coverage.data.resize(5);
		coverage.data[0] = ros::Time::now().toSec();
		coverage.data[1] = occupied_cells;
		coverage.data[2] = free_cells;
		coverage.data[3] = missing_cells;
		coverage.data[4] = total_cells;
	
		coverage_pub.publish(coverage);

        std_msgs::Float64MultiArray msg;
        msg.data.resize(uavs_in_range.size() + 1);
        msg.data[0] = ros::Time::now().toSec();
        int i = 1;
        for (const auto& uav_id : uavs_in_range)
        {
            msg.data[i] = static_cast<double>(uav_id);
            ++i;
        }

        uavs_in_range_pub.publish(msg);
    }

    ros::Subscriber octomap_sub;
    ros::Subscriber planner_sub;
    ros::Subscriber odom_sub;
    ros::Publisher octomap_pub;
    ros::Publisher frontier_pub;
    ros::Publisher frontier_parent_pub;
    ros::Publisher cluster_pub;
    ros::Publisher waypoint_pub;
    ros::Publisher coverage_pub;
    ros::Publisher uavs_in_range_pub;
    ros::ServiceClient hover_srv;
    ros::ServiceClient planner_srv;

    std::unique_ptr<octomap::OcTree> octree;
    bool path_available;
    octomap::point3d pos;

    octomap::KeySet frontiers;
    octomap::KeySet frontier_parents;
    octomap::KeySet clusters;
    std::vector<octomap::point3d> candidates;
    octomap::point3d waypoint;
    float waypoint_score;
    std::unordered_map<octomap::OcTreeKey, int, octomap::OcTreeKey::KeyHash> unreached;
    std::set<int> uavs_in_range;

    int uav_index;
    int num_uavs;

private:

    double r_exp;
    double bandwidth;
    double sensor_range;
    double lambda;
    double waypoint_distance_threshold;

    double exploration_min_x;
    double exploration_max_x;
    double exploration_min_y;
    double exploration_max_y;
    double exploration_min_z;
    double exploration_max_z;
};

class FrontierExploration {
public:
    FrontierExploration(ros::NodeHandle& nh, ros::NodeHandle& nh_private, int num_uavs)
    {
        uavs.reserve(num_uavs);

        for(int i = 0; i < num_uavs; i++)
        {
            std::string uav_name = "uav" + std::to_string(i + 1);
            ROS_INFO("Initializing UAV %s", uav_name.c_str());
            uavs.emplace_back(nh, nh_private, uav_name, i + 1);
        }

        nh_private.param("just_detection", just_detection, false);
        nh_private.param("merge_distance", merge_distance, 20.0);
        nh_private.param("min_score", min_score, 0.2);
        nh_private.param("r_exp", r_exp, 0.5);
        nh_private.param("update_rate", update_rate, 1.0);

    }

    void run()
	{
        ros::Rate loop_rate(10);

        while (ros::ok())
        {
            ros::spinOnce();
            shareInfo();
            assigned_waypoints.clear();

            for(auto& uav : uavs)
            {
                if(uav.octree)
                {
                    uav.frontierDetection();
                    uav.searchParents();
                    uav.frontierClustering();
                    uav.waypoint = uav.frontierEvaluation(assigned_waypoints);
                    assigned_waypoints.push_back(uav.waypoint);
                    uav.publishWaypoint();
                    if(!just_detection)
                        pathPlanner(uav);
                }
                else
                {
                    ROS_INFO("Octree not initialized for UAV %d", uav.uav_index);
                }
            }

            loop_rate.sleep();
        }
    }

    void shareInfo()
    {
        for(auto& uav1 : uavs)
        {
            for(auto& uav2 : uavs)
            {
                if(uav1.uav_index == uav2.uav_index)
                    continue;

                double distance = (uav1.pos - uav2.pos).norm();

                if(distance <= merge_distance && uav1.octree && uav2.octree)
                {
                    for(auto it = uav2.octree->begin_leafs(), end = uav2.octree->end_leafs(); it != end; ++it)
                    {
                        uav1.octree->updateNode(it.getKey(), it->getValue());
                    }
                }
            }
        }
    }

    void pathPlanner(UAV& uav)
    {
        bool new_uav_in_range = false;
        bool waypoint_reached = false;
        octomap::OcTreeKey current_key = uav.octree->coordToKey(uav.waypoint);
        ros::Time start_time = ros::Time::now();

        if(uav.waypoint_score < min_score)
        {
            std_srvs::Trigger srv;
            if (uav.hover_srv.call(srv))
            {
                if (srv.response.success)
                    ROS_INFO("Hover activated");
                else
                    ROS_ERROR("Failed to call hover service");
            }
            return;
        }

        while (!new_uav_in_range && !waypoint_reached && (ros::Time::now() - start_time).toSec() < update_rate)
        {
            ros::spinOnce();
            shareInfo();

            mrs_msgs::Vec4 srv;
            srv.request.goal[0] = uav.waypoint.x();
            srv.request.goal[1] = uav.waypoint.y();
            srv.request.goal[2] = uav.waypoint.z();
            srv.request.goal[3] = 0;

            if (!uav.planner_srv.call(srv))
            {
                ROS_ERROR("Failed to call planner service");
                return;
            }

            if(!uav.path_available)
            {
                uav.unreached[current_key] += 1;
                ROS_INFO("Added waypoint to unreached. Current count: %d", uav.unreached[current_key]);
                break;
            }

            for (auto& other_uav : uavs)
            {
                if (other_uav.uav_index != uav.uav_index && (other_uav.pos - uav.pos).norm() <= merge_distance)
                {
                    if (uav.uavs_in_range.find(other_uav.uav_index) == uav.uavs_in_range.end())
                    {
                        uav.uavs_in_range.insert(other_uav.uav_index);
                        ROS_INFO("UAV %d is in range of UAV %d", other_uav.uav_index, uav.uav_index);
                        break;
                    }
                }

                else if(other_uav.uav_index != uav.uav_index && (other_uav.pos - uav.pos).norm() > merge_distance)
                {
                    if (uav.uavs_in_range.find(other_uav.uav_index) != uav.uavs_in_range.end())
                    {
                        uav.uavs_in_range.erase(other_uav.uav_index);
                        ROS_INFO("UAV %d is no longer in range of UAV %d", other_uav.uav_index, uav.uav_index);
                        break;
                    }
                }
            }

            if((uav.pos - uav.waypoint).norm() < r_exp)
            {
                waypoint_reached = true;
                ROS_INFO("Waypoint reached");
                break;
            }
        }
    }

private:
    bool just_detection;
    double merge_distance;
    double min_score;
    double r_exp;
    double update_rate;

    std::vector<UAV> uavs;
    std::vector<octomap::point3d> assigned_waypoints;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "frontier_exploration");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    int num_uavs;
    nh_private.param("num_uavs", num_uavs, 1);

    FrontierExploration fe(nh, nh_private, num_uavs);

    fe.run();

    return 0;
}
