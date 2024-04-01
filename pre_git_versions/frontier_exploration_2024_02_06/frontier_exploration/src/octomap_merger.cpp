#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>

octomap::OcTree* octree1 = NULL;
octomap::OcTree* octree2 = NULL;
octomap::point3d pos1;
octomap::point3d pos2;
double distanceThreshold = 10.0;

ros::Publisher octomap_pub;

void octomapCallback1(const octomap_msgs::Octomap::ConstPtr& msg)
{
    try
    {
        octree1 = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(*msg);
    } 
    catch (std::exception& e)
    {
        ROS_ERROR("Failed to convert OctoMap message to OcTree: %s", e.what());
    }
}

void octomapCallback2(const octomap_msgs::Octomap::ConstPtr& msg)
{
    try
    {
        octree2 = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(*msg);
    }
    catch (std::exception& e)
    {
        ROS_ERROR("Failed to convert OctoMap message to OcTree: %s", e.what());
    }
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

bool proximityCheck()
{
    double distance = (pos1 - pos2).norm();
    if (distance <= distanceThreshold) return true;
    else return false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "octomap_merger");
    ros::NodeHandle nh;

    ros::Subscriber octomap_sub1 = nh.subscribe("uav1/octomap_server/octomap_global_binary", 1, octomapCallback1);
    ros::Subscriber octomap_sub2 = nh.subscribe("uav2/octomap_server/octomap_global_binary", 1, octomapCallback2);
    ros::Subscriber odom_sub1 = nh.subscribe("uav1/estimation_manager/odom_main", 1, odomCallback1);
    ros::Subscriber odom_sub2 = nh.subscribe("uav2/estimation_manager/odom_main", 1, odomCallback2);

    octomap_pub = nh.advertise<octomap_msgs::Octomap>("merged_octomap", 1);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        if (octree1 && octree2)
        {
            if (proximityCheck())
            {
                try
                {
                    octree1->expand();
                    for(octomap::OcTree::leaf_iterator it = octree2->begin_leafs(), end=octree2->end_leafs(); it!= end; ++it)
                    {
                        octree1->updateNode(it.getKey(), it->getValue());
                    }

                    // Convert the merged OcTree to an OctoMap message
                    octomap_msgs::Octomap msg;
                    octomap_msgs::binaryMapToMsg(*octree1, msg);

                    // Set the frame_id field of the header
                    msg.header.frame_id = "common_origin";

                    // Publish the merged OctoMap
                    octomap_pub.publish(msg);
                }
                catch (std::exception& e)
                {
                    ROS_ERROR("Failed to merge or publish OctoMaps: %s", e.what());
                }
            }
            // else
            // {
            //     try
            //     {
            //         octomap_msgs::Octomap msg1, msg2;
            //         octomap_msgs::binaryMapToMsg(*octree1, msg1);
            //         octomap_msgs::binaryMapToMsg(*octree2, msg2);

            //         // Set the frame_id field of the header
            //         msg1.header.frame_id = "common_origin";
            //         msg2.header.frame_id = "common_origin";

            //         // Publish the merged OctoMap
            //         octomap_pub1.publish(msg1);
            //         octomap_pub2.publish(msg2);
            //     }
            //     catch (std::exception& e)
            //     {
            //         ROS_ERROR("Failed to merge or publish OctoMaps: %s", e.what());
            //     }
            // }

            delete octree1;
            delete octree2;
            octree1 = NULL;
            octree2 = NULL;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
