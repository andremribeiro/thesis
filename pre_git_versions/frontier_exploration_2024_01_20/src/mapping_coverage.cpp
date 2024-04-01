#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>

class MappingCoverage
{
public:
    MappingCoverage()
    {
        ros::NodeHandle nh;
        octomap_sub = nh.subscribe<octomap_msgs::Octomap>("uav1/octomap_server/octomap_global_binary", 10, &MappingCoverage::octomapCallback, this);
    }

    void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg)
    {
        octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(*msg));

        if (!octree)
        {
            ROS_ERROR("Failed to convert Octomap message to OcTree");
            return;
        }

        // Calculate the total number of cells in the area
        double resolution = octree->getResolution();  // Resolution of the OctoMap
        int total_cells = (40 * 40 * 10) / resolution;  // Total number of cells in the area        int mapped_cells = 0;
        int mapped_cells = 0;

        // Define the size of the bounding box (20 units in each direction from the origin)
        double size = 20.0;

        // Create the minimum and maximum points of the bounding box
        octomap::point3d min(-size, -size, 0);
        octomap::point3d max(size, size, 10);

        for(octomap::OcTree::leaf_bbx_iterator it = octree->begin_leafs_bbx(min, max), end=octree->end_leafs_bbx(); it!= end; ++it)
        {
            total_cells++;
            if(octree->isNodeOccupied(*it))
            {
                // Node is occupied
                mapped_cells++;
            }
            else if(it->getOccupancy() < octree->getOccupancyThres())
            {
                // Node is free
                mapped_cells++;
            }
        }

        float percentage_mapped = 100.0 * mapped_cells / total_cells;

        ROS_INFO("Mapped: %f", percentage_mapped);

        delete octree;
    }

private:
    ros::Subscriber octomap_sub;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mapping_coverage");

    MappingCoverage mapping_coverage;

    ros::spin();

    return 0;
}
