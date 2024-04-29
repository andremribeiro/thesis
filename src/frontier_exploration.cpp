#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <mrs_msgs/Vec4.h>
#include <nav_msgs/Odometry.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>
#include <octomap/OcTree.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Trigger.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <cmath>
#include <functional>
#include <memory>
#include <set>
#include <unordered_map>
#include <vector>

#include <frontier_exploration/clusteringAlgorithm.h>

class UAV {
public:
    UAV(ros::NodeHandle& nh, ros::NodeHandle& nh_private, int uav_id, int num_uavs) : uav_id(uav_id), num_uavs(num_uavs)
    {
        std::string uav_name = "/uav" + std::to_string(uav_id);
        ROS_INFO("Initializing UAV %d", uav_id);

        octomap_sub = nh.subscribe<octomap_msgs::Octomap>(uav_name + "/octomap_binary", 1, std::bind(&UAV::octomapCallback, this, std::placeholders::_1));
        odom_sub = nh.subscribe<nav_msgs::Odometry>(uav_name + "/estimation_manager/odom_main", 1, std::bind(&UAV::odomCallback, this, std::placeholders::_1));
        octomap_pub = nh.advertise<octomap_msgs::Octomap>(uav_name + "/shared_octomap", 1);
        frontier_pub = nh.advertise<visualization_msgs::Marker>(uav_name + "/frontier_markers", 1);
        cluster_pub = nh.advertise<visualization_msgs::Marker>(uav_name + "/cluster_markers", 1);
        waypoint_pub = nh.advertise<geometry_msgs::PointStamped>(uav_name + "/waypoint", 1);
        results_pub = nh.advertise<std_msgs::Float64MultiArray>(uav_name + "/results", 1);
        planner_srv = nh.serviceClient<mrs_msgs::Vec4>(uav_name + "/octomap_planner/goto");
        hover_srv = nh.serviceClient<mrs_msgs::Vec4>(uav_name + "/octomap_planner/stop");

        nh_private.param("bandwidth", bandwidth, 1.0);
        nh_private.param("comms_range", comms_range, 40.0);
        nh_private.param("exploration_min_x", exploration_min_x, -30.0);
        nh_private.param("exploration_max_x", exploration_max_x, 30.0);
        nh_private.param("exploration_min_y", exploration_min_y, -30.0);
        nh_private.param("exploration_max_y", exploration_max_y, 30.0);
        nh_private.param("exploration_min_z", exploration_min_z, 10.0);
        nh_private.param("exploration_max_z", exploration_max_z, 0.0);
        nh_private.param("lambda", lambda, 0.1);
        nh_private.param("update_rate", update_rate, 1.0);
        nh_private.param("min_score", min_score, 1.0);
        nh_private.param("r_exp", r_exp, 1.0);
        nh_private.param("sensor_range", sensor_range, 20.0);
        nh_private.param("waypoint_distance", waypoint_distance, 10.0);

        for(int i = 1; i <= num_uavs; i++) {
            if(i == uav_id) continue;

            std::string other_uav_name = "/uav" + std::to_string(i);
            ros::Subscriber other_octomap_sub = nh.subscribe<octomap_msgs::Octomap>(other_uav_name + "/shared_octomap", 1, std::bind(&UAV::sharedOctomapCallback, this, std::placeholders::_1, i));
            ros::Subscriber other_odom_sub = nh.subscribe<nav_msgs::Odometry>(other_uav_name + "/estimation_manager/odom_main", 1, std::bind(&UAV::sharedOdomCallback, this, std::placeholders::_1, i));
            ros::Subscriber other_waypoint_sub = nh.subscribe<geometry_msgs::PointStamped>(other_uav_name + "/waypoint", 1, std::bind(&UAV::sharedWaypointCallback, this, std::placeholders::_1, i));
            shared_octomap_subs.push_back(other_octomap_sub);
            shared_odom_subs.push_back(other_odom_sub);
            shared_waypoint_subs.push_back(other_waypoint_sub);
        }
    }

    void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg)
    {
        octomap::OcTree* local_octree = dynamic_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(*msg));

        if(local_octree)
        {
            local_octree->expand();
            if(octree)
            {
                for(auto it = local_octree->begin(), end = local_octree->end(); it != end; ++it)
                {
                    octomap::OcTreeNode* node = octree->search(it.getKey());
                    if(node != NULL)
                        node->setLogOdds((node->getLogOdds() + it->getLogOdds()) / 2.0);
                    else
                        octree->updateNode(it.getKey(), it->getLogOdds());
                }
                
                octomap_msgs::Octomap msg_out;
                octomap_msgs::binaryMapToMsg(*octree, msg_out);

                msg_out.header.frame_id = "common_origin";
                octomap_pub.publish(msg_out);
            }
            else
                octree.reset(local_octree);
        }
        else 
            ROS_ERROR("Failed to convert octomap message to OcTree for UAV %d", uav_id);

        publishResults();
    }
    
    void odomCallback(const nav_msgs::OdometryConstPtr& msg)
    {     
        pos.x() = msg->pose.pose.position.x;
        pos.y() = msg->pose.pose.position.y;
        pos.z() = msg->pose.pose.position.z;
    }
    
    void sharedOctomapCallback(const octomap_msgs::Octomap::ConstPtr& msg, int uav_id)
    {
        double distance = (pos - other_pos[uav_id]).norm();

        if(distance <= comms_range)
        {
            octomap::OcTree* shared_octree = dynamic_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(*msg));

            if(shared_octree)
            {
                shared_octree->expand();
                if(octree)
                {
                    for(auto it = shared_octree->begin(), end = shared_octree->end(); it != end; ++it)
                    {
                        octomap::OcTreeNode* node = octree->search(it.getKey());
                        if(node != NULL)
                            node->setLogOdds((node->getLogOdds() + it->getLogOdds()) / 2.0);
                        else
                            octree->updateNode(it.getKey(), it->getLogOdds());
                    }
                    
                    octomap_msgs::Octomap msg_out;
                    octomap_msgs::binaryMapToMsg(*octree, msg_out);

                    msg_out.header.frame_id = "common_origin";
                    octomap_pub.publish(msg_out);
                }
                else
                    octree.reset(shared_octree);
            }
            else 
                ROS_ERROR("Failed to convert shared octomap message to OcTree");
        }
    }

    void sharedOdomCallback(const nav_msgs::OdometryConstPtr& msg, int other_uav_id)
    {
        distance2uavs.erase(other_uav_id);

        octomap::point3d shared_pos;
        shared_pos.x() = msg->pose.pose.position.x;
        shared_pos.y() = msg->pose.pose.position.y;
        shared_pos.z() = msg->pose.pose.position.z;
        other_pos[other_uav_id] = shared_pos;

        double distance = (pos - shared_pos).norm();
        distance2uavs[other_uav_id] = distance;
    }

    void sharedWaypointCallback(const geometry_msgs::PointStamped::ConstPtr& msg, int other_uav_id)
    {
        octomap::point3d shared_waypoint;
        shared_waypoint.x() = msg->point.x;
        shared_waypoint.y() = msg->point.y;
        shared_waypoint.z() = msg->point.z;
        other_waypoints[other_uav_id] = shared_waypoint;
    }

    void frontierDetection()
    {
        frontiers.clear();
        
        ros::WallTime start_time = ros::WallTime::now();

        bool unknown = false;
        bool occupied = false;
        int depth = octree->getTreeDepth() - std::log2(r_exp / octree->getResolution());

        std::vector <octomap::point3d> neighbors;

        for(octomap::OcTree::iterator it = octree->begin(depth), end=octree->end_leafs(); it!= end; ++it)
        {
            octomap::OcTreeKey current_key = it.getKey();
            octomap::point3d current_point = octree->keyToCoord(current_key, depth);

            if (!isWithinExplorationBounds(current_point))
                continue;

            octomap::OcTreeNode* current_node = octree->search(current_key, depth);

            if(!octree->isNodeOccupied(current_node))
            {
                unknown = false;
                occupied = false;

                getNeighbor(current_key, neighbors, depth);

                for (std::vector<octomap::point3d>::iterator iter = neighbors.begin(); iter != neighbors.end(); iter++)
                {
                    octomap::point3d ngbr_point =* iter;

                    if(!isWithinExplorationBounds(ngbr_point))
                        continue;

                    octomap::OcTreeNode* ngbr_node = octree-> search(ngbr_point, depth);
                    if(ngbr_node == NULL)
                        unknown = true;
                    else if(octree->isNodeOccupied(ngbr_node))
                        occupied = true;            
                }

                if(unknown && !occupied)
                    frontiers.insert(current_key);
            }
        }
        ROS_INFO("UAV %d found %zu frontiers in %f seconds", uav_id, frontiers.size(), (ros::WallTime::now() - start_time).toSec());

        publishMarker(frontiers, "frontiers", 0.75, frontier_pub);
    }

    bool isWithinExplorationBounds(const octomap::point3d& point) {
        return (point.x() >= exploration_min_x && point.x() <= exploration_max_x && point.y() >= exploration_min_y && point.y() <= exploration_max_y && point.z() >= exploration_min_z && point.z() <= exploration_max_z);
    }

    void getNeighbor(const octomap::OcTreeKey& key, std::vector<octomap::point3d>& neighbors, int depth) 
    {
        neighbors.clear();
        octomap::OcTreeKey neighbor_key;
        int depth_diff = octree->getTreeDepth() - depth + 1;

        for (int dx = -depth_diff; dx <= depth_diff; dx += depth_diff) {
            for (int dy = -depth_diff; dy <= depth_diff; dy += depth_diff) {
                for (int dz = -depth_diff; dz <= depth_diff; dz += depth_diff) {
                    if (dx == 0 && dy == 0 && dz == 0)
                        continue;

                    neighbor_key = key;
                    neighbor_key[0] += dx;
                    neighbor_key[1] += dy;
                    neighbor_key[2] += dz;

                    octomap::point3d query = octree->keyToCoord(neighbor_key, depth);
                    neighbors.push_back(query);
                }
            }
        }
    }

    void frontierClustering()
    {
		clusters.clear();

        ros::WallTime start_time = ros::WallTime::now();
		
		std::vector<geometry_msgs::Point> original_points {};
		std::vector<geometry_msgs::Point> clustered_points {};

		keyToPointVector(frontiers, original_points);
		MSCluster *cluster = new MSCluster();
		cluster->getMeanShiftClusters(original_points, clustered_points, bandwidth);
		std::vector<octomap::OcTreeKey> clusters_key {};
		pointVectorToKey(clustered_points, clusters_key);

		for (std::vector<octomap::OcTreeKey>::iterator iter = clusters_key.begin(); iter != clusters_key.end(); iter++)
			clusters.insert(*iter);

		delete cluster;

        ROS_INFO("UAV %d found %zu clusters in %f seconds", uav_id, clusters.size(), (ros::WallTime::now() - start_time).toSec());

        publishMarker(clusters, "clusters", 1.0, cluster_pub);
	}

    void keyToPointVector(octomap::KeySet& frontierCells, std::vector<geometry_msgs::Point>& original_points)
    {
		for(octomap::KeySet::iterator iter = frontierCells.begin(), end = frontierCells.end(); iter!= end; ++iter)
		{
            octomap::OcTreeKey temp_key;
            temp_key = *iter;

            octomap::point3d temp_point;
            temp_point = octree->keyToCoord(temp_key);

            geometry_msgs::Point temp_geom_point;
            temp_geom_point.x = temp_point.x();
            temp_geom_point.y = temp_point.y();
            temp_geom_point.z = temp_point.z();

            original_points.push_back(temp_geom_point);
		}
	}

    void pointVectorToKey(std::vector<geometry_msgs::Point>& points, std::vector<octomap::OcTreeKey>& clusters_key)
	{
		for (int i = 0; i < points.size(); i++)
		{
			octomap::point3d temp_point;
			temp_point.x() = points[i].x;
			temp_point.y() = points[i].y;
			temp_point.z() = points[i].z;

			octomap::OcTreeKey temp_key;
			if (!octree->coordToKeyChecked(temp_point, temp_key)) 
			{
				OCTOMAP_ERROR_STR("Error in search: [" << temp_point << "] is out of OcTree bounds!");
				return;
			} 
			clusters_key.push_back(temp_key);
		}
	}

    void frontierEvaluation()
    {
        ros::WallTime start_time = ros::WallTime::now();

        double max_score = -std::numeric_limits<double>::infinity();
        octomap::point3d best_candidate;

        for(octomap::KeySet::iterator iter = clusters.begin(), end = clusters.end(); iter != end; ++iter)
        {
            octomap::point3d candidate = octree->keyToCoord(*iter);

            bool skip_candidate = false;
            for(const auto& other_waypoint : other_waypoints) {
                if(other_waypoint.first < uav_id) {
                    double distance2other_waypoint = (candidate - other_waypoint.second).norm();
                    if(distance2other_waypoint <= waypoint_distance) {
                        skip_candidate = true;
                        continue;
                    }
                }
            }

            if(skip_candidate) continue;

            double distance_sum = 0.0;
            for(const auto& other_uav : distance2uavs) {
                distance_sum += other_uav.second;
            }
            
            double distance = calculateDistance(pos, candidate);
            double info_gain = calculateInfoGain(candidate);
            double score;
            
            if(distance2uavs.size() == 0)
                score = 100.0 * info_gain * exp(-lambda * distance);
            else
                score = 100.0 * info_gain * exp(-lambda * (distance + distance2uavs.size() / distance_sum));

            if(score > max_score)
            {
                max_score = score;
                best_candidate = candidate;
            }
        }

        ROS_INFO("UAV %d selected waypoint (%f, %f, %f) with score %f in %f seconds", uav_id, best_candidate.x(), best_candidate.y(), best_candidate.z(), max_score, (ros::WallTime::now() - start_time).toSec());

        waypoint_score = max_score;
        waypoint = best_candidate;

        publishWaypoint();
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

                    if(abs(elevation_angle) <= M_PI / 4)
                    {
                        total++;
                        if(!octree->search(dx, dy, dz)) unknown++;
                    }
                }
            }
        }   
        return (double)unknown / (double)total;
    }

    void pathPlanner()
    {
        bool new_uav = false;
        bool waypoint_reached = false;
        ros::Time start_time = ros::Time::now();

        if(waypoint_score < min_score)
        {
            ROS_INFO("UAV %d has no suitable waypoint", uav_id);

            std_srvs::Trigger srv;

            if(!hover_srv.call(srv))
                ROS_ERROR("Failed to call hover service for UAV %d", uav_id);

            return;
        }

        while(!new_uav && !waypoint_reached && ros::Time::now() - start_time < ros::Duration(update_rate))
        {
            ros::spinOnce();

            for(const auto& other_uav : distance2uavs)
            {
                if(distance2uavs[other_uav.first] <= comms_range)
                {
                    if(uavs_in_range.find(other_uav.first) == uavs_in_range.end())
                    {
                        uavs_in_range.insert(other_uav.first);
                        ROS_INFO("UAV %d is in range of UAV %d", uav_id, other_uav.first);
                        new_uav = true;
                        break;
                    }
                } 
                else
                {
                    if(uavs_in_range.find(other_uav.first) != uavs_in_range.end())
                    {
                        ROS_INFO("UAV %d is out of range of UAV %d", uav_id, other_uav.first);
                        uavs_in_range.erase(other_uav.first);
                    }
                }
            }

            if(new_uav) continue;

            double distance2waypoint = calculateDistance(pos, waypoint);

            if(distance2waypoint <= r_exp)
            {
                ROS_INFO("UAV %d reached waypoint", uav_id);
                waypoint_reached = true;
                continue;
            }

            mrs_msgs::Vec4 srv;
            srv.request.goal[0] = waypoint.x();
            srv.request.goal[1] = waypoint.y();
            srv.request.goal[2] = waypoint.z();
            srv.request.goal[3] = 0.0;

            if(!planner_srv.call(srv))
                ROS_ERROR("Failed to call path planner service for UAV %d", uav_id);

        }
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
        } else if (ns == "clusters") {
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
        }

        for (const octomap::KeySet::value_type& key : data)
        {
            octomap::point3d point = octree->keyToCoord(key);
            geometry_msgs::Point marker_point;
            marker_point.x = point.x();
            marker_point.y = point.y();
            marker_point.z = point.z();
            marker.points.push_back(marker_point);
        }
        pub.publish(marker);
    }

    void publishWaypoint()
    {
        geometry_msgs::PointStamped waypoint_point;
        waypoint_point.header.frame_id = "common_origin";
        waypoint_point.header.stamp = ros::Time::now();
        waypoint_point.point.x = waypoint.x();
        waypoint_point.point.y = waypoint.y();
        waypoint_point.point.z = waypoint.z();
        waypoint_pub.publish(waypoint_point);
    }

    void publishResults()
    {
        double free_cells = 0.0;
        double occupied_cells = 0.0;
        double total_cells = (exploration_max_x - exploration_min_x) * (exploration_max_y - exploration_min_y) * (exploration_max_z - exploration_min_z) / std::pow(octree->getResolution(), 3);

        octree->expand();
        for(octomap::OcTree::iterator it = octree->begin(), end = octree->end(); it != end; ++it)
        {
            if(isWithinExplorationBounds(it.getCoordinate()))
            {
                if(!octree->isNodeOccupied(*it))
                    free_cells++;
                else if(octree->isNodeOccupied(*it))
                    occupied_cells++;
            }
        }

        double known_cells = free_cells + occupied_cells;
        double unknown_cells = total_cells - known_cells;
        double exploration_rate = 100 * known_cells / total_cells;

        // ROS_INFO("Free cells: %.2f", free_cells);
        // ROS_INFO("Occupied cells: %.2f", occupied_cells);
        // ROS_INFO("Unknown cells: %.2f", unknown_cells);
        // ROS_INFO("Total cells: %.2f", total_cells);

        ROS_INFO("UAV %d has explored %.2f%% of the environment", uav_id, exploration_rate);

        std_msgs::Float64MultiArray results;
        results.data.push_back(ros::Time::now().toSec());
        results.data.push_back(pos.x());
        results.data.push_back(pos.y());
        results.data.push_back(pos.z());
        results.data.push_back(free_cells);
        results.data.push_back(occupied_cells);
        results.data.push_back(unknown_cells);
        results.data.push_back(total_cells);
        results.data.push_back(frontiers.size());
        results.data.push_back(clusters.size());
        results.data.push_back(waypoint.x());
        results.data.push_back(waypoint.y());
        results.data.push_back(waypoint.z());
        results.data.push_back(waypoint_score);

        results_pub.publish(results);
    }

    std::unique_ptr<octomap::OcTree> octree;
    octomap::KeySet frontiers;
    octomap::KeySet clusters;
    octomap::point3d waypoint;
    double waypoint_score;


private:
    ros::Subscriber octomap_sub;
    ros::Subscriber odom_sub;
    ros::Publisher octomap_pub;
    ros::Publisher frontier_pub;
    ros::Publisher cluster_pub;
    ros::Publisher waypoint_pub;
    ros::Publisher results_pub;
    ros::ServiceClient planner_srv;
    ros::ServiceClient hover_srv;
    std::vector<ros::Subscriber> shared_octomap_subs;
    std::vector<ros::Subscriber> shared_odom_subs;
    std::vector<ros::Subscriber> shared_waypoint_subs;

    octomap::point3d pos;
    std::map<int, octomap::point3d> other_pos;
    std::map<int, double> distance2uavs;
    std::map<int, octomap::point3d> other_waypoints;
    std::set<int> uavs_in_range;

    int uav_id;
    int num_uavs;
    double bandwidth;
    double comms_range;
    double exploration_min_x;
    double exploration_max_x;
    double exploration_min_y;
    double exploration_max_y;
    double exploration_min_z;
    double exploration_max_z;
    double lambda;
    double update_rate;
    double min_score;
    double r_exp;
    double sensor_range;
    double waypoint_distance;
};

class FrontierExploration {
public:
    FrontierExploration(ros::NodeHandle& nh, ros::NodeHandle& nh_private, int uav_id, int num_uavs)
    {
        uav = new UAV(nh, nh_private, uav_id, num_uavs);
    }

    void run()
	{
        ros::Rate loop_rate(1);

        while(ros::ok())
        {
            ros::spinOnce();
            if(uav->octree)
            {
                uav->frontierDetection();
                if(uav->frontiers.size() != 0)
                    uav->frontierClustering();
                if(uav->clusters.size() != 0)
                    uav->frontierEvaluation();
                uav->pathPlanner();
            }
            loop_rate.sleep();
        }
    }

private:
    UAV* uav;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "frontier_exploration");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    int uav_id, num_uavs;
    nh_private.param("uav_id", uav_id, 1);
    nh_private.param("num_uavs", num_uavs, 1);

    FrontierExploration fe(nh, nh_private, uav_id, num_uavs);
    fe.run();

    return 0;
}
