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
#include "metrics.h"
#include <pcl/console/time.h> // TicToc

#include "json.hpp"
using json = nlohmann::json;

int main (int argc, char* argv[])
{


    //ALIGNMENT
    // For reproducability
    srand(100);

    std::string path = "../../data/Dataset_1/D1/";

    std::ifstream f(path + std::string("splits/info.json"));
    auto data = json::parse(f);
    std::vector<int> hard_idxs = data["hard"];
    std::vector<int> easy_idxs = data["easy"];


    Dataset dataset1_d1(path);
    std::cout << "Dataset Loaded from " << path << std::endl;
    std::vector<double> time_list;

    std::cout << "Dataset Size: " << dataset1_d1.c_poses.size() <<std::endl;

    // SINGLE FRAME RUN FOR TESTING
    SimplePipeline pipeline;
    Frame frame = dataset1_d1.getFrame(69);
    Eigen::Matrix4d transform = pipeline.guess_v_pose(frame);
    //vizulization stuff
    PointCloudT::Ptr cloud_c_new (new PointCloudT); 
    pcl::transformPointCloud (*frame.cloud_c, *cloud_c_new, transform);
    create_visualizer(std::string("Demo Visualizer"), frame.cloud_i, frame.cloud_c, cloud_c_new); 


    //READ FULL DATASET RUN
    // SimplePipeline pipeline(dataset1_d1);
    // pipeline.run();

    // //METRICS
    // print_metrics(pipeline, easy_idxs, hard_idxs);



}