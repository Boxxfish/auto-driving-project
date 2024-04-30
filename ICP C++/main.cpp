#include <iostream>
#include <string>
#include <random>
#include <typeinfo>
#include <fstream>
#include <sstream>
#include <cmath> // for radians conversion

#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
#include <Eigen/Geometry> // Include this for additional Eigen functionalities

#include "visualizer.h"
#include "dataset.h"
#include "frame.h"
#include "pipeline.h"

int main (int argc, char* argv[])
{

    StdPipeline pipeline;


    std::string path = "../../data/Dataset_1/D1/";
    auto dataset1_d1 = new Dataset(path);

    std::cout << "Dataset Loaded from " << path << std::endl;

    Frame f1 = dataset1_d1->getFrame(1);
    std::cout << "Frame Loaded " << path << std::endl;

    // Eigen::Matrix4d c = pipeline.guess_v_pose(f1);

    // std::cout << c << std::endl;

    // std::cout << "Dataset poses vector: \n" << dataset1_d1->c_poses_corrected[1] << std::endl;
    // dataset1_d1->c_poses_corrected[1] = c;
    
    PointCloudT::Ptr cloud_c_new (new PointCloudT); 

    std::cout << "pose c: \n" << f1.pose_c << std::endl;
    pcl::transformPointCloud (*f1.cloud_c, *cloud_c_new, f1.pose_c);
    std::cout << "pose i: \n" << f1.pose_i << std::endl;

    pcl::transformPointCloud (*f1.cloud_i, *f1.cloud_i, f1.pose_i);

    create_visualizer(std::string("Demo Visualizer"), f1.cloud_i, f1.cloud_c, cloud_c_new); 
}