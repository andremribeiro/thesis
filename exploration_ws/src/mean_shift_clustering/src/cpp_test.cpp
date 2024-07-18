#include <stdio.h>
#include <stdlib.h>
#include <mean_shift_clustering/MeanShiftAlgorithm.h>
#include <ros/ros.h>
#include <string>
#include <fstream>
#include <vector>
#include <sstream> // std::stringstream
#include <geometry_msgs/Point.h>

using namespace std;

vector<geometry_msgs::Point> load_points()
{
    std::vector<geometry_msgs::Point> result;
    geometry_msgs::Point point;
    string line;
    ifstream csvfile;
    csvfile.open("input.csv");
    int i = 0;
    double x, y, z;
    while (!csvfile.eof() ){//loops until the end of the file
        getline(csvfile, line, ',');//read a string until the next comma
        x = atof(line.c_str());
        point.x = x; 
        getline(csvfile, line, ',');
        y = atof(line.c_str());
        point.y = y; 
        getline(csvfile, line);
        z = atof(line.c_str());
        point.z = z; 
        result.push_back(point);
    }
    csvfile.close();
    return result;
}

void print_points(vector<geometry_msgs::Point> points)
{
    for(int i = 0; i< points.size(); i++){
            printf("%f ", points[i].x);
            printf("%f ", points[i].y);
            printf("%f ", points[i].z);
        printf("\n");
    }
}

int main(int argc, char **argv)
{
    ros::WallTime startTime_evaluation = ros::WallTime::now();
    ros::init(argc, argv, "mean_shift_clustering_node");
    MeanShift *msp = new MeanShift();
    double kernel_bandwidth = 2.55;
    vector<geometry_msgs::Point> points = load_points();
    // print_points(points);
    vector<Cluster> clusters = msp->cluster(points, kernel_bandwidth);
    double total_time_evaluation = (ros::WallTime::now() - startTime_evaluation).toSec();
	cout <<"Mean shift used total "<< total_time_evaluation <<" sec"<<endl;
    FILE *fp = fopen("output.csv", "w");
    if(!fp){
        perror("Couldn't write result.csv");
        exit(0);
    }

  
    for(int cluster = 0; cluster < clusters.size(); cluster++) {
            fprintf(fp, "%f,%f,%f,%s\n", clusters[cluster].mode.x, 
            clusters[cluster].mode.y,
            clusters[cluster].mode.z,
            "c");
      for(int point = 0; point < clusters[cluster].original_points.size(); point++){
            fprintf(fp, "%f,%f,%f,%s\n", clusters[cluster].original_points[point].x, 
                clusters[cluster].original_points[point].y,
                clusters[cluster].original_points[point].z,
                "o");
            
             fprintf(fp, "%f,%f,%f,%s\n", clusters[cluster].shifted_points[point].x, 
                clusters[cluster].shifted_points[point].y,
                clusters[cluster].shifted_points[point].z,
                "s");
        }
    }
    std::fclose(fp);

    return 0;
}
