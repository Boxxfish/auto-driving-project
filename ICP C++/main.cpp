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
#include <pcl/console/time.h> // TicToc
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
#include <Eigen/Geometry> // Include this for additional Eigen functionalities

#include "visualizer.h"
#include "dataset.h"
#include "pipeline.h"
#include "metrics.h"
#include <pcl/console/time.h> // TicToc

int main(int argc, char *argv[])
{

    // ALIGNMENT
    //  For reproducability
    srand(100);

    // // SINGLE FRAME RUN FOR TESTING
    StdPipeline pipeline(true, 8);
    std::string path = "../../data/Dataset_1/D2/";
    Dataset dataset1_d1(path);
    int frame_num = 69; //you can exchange frame 69 with any frame you want
    std::optional<Eigen::Matrix4d> transform = pipeline.guess_v_pose(dataset1_d1.frames[frame_num],dataset1_d1.i_pose); 
    //vizulization stuff
    PointCloudT::Ptr cloud_c_new (new PointCloudT);
    pcl::transformPointCloud (*dataset1_d1.frames[frame_num].cloud_c, *cloud_c_new, *transform);
    create_visualizer(std::string("Demo Visualizer"), dataset1_d1.frames[frame_num].cloud_i, dataset1_d1.frames[frame_num].cloud_c, cloud_c_new);

    // METRICS

    //TEST dataset_1/D1 full pipeline
    // std::string path = "../../data/Dataset_1/D2/";
    // StdPipeline pipeline(true, 8);
    // Dataset dataset(path);
    // print_metrics(pipeline, dataset);


    //TEST dataset_1/D1 no rotaion pipeline (uses ground truth rotation)
    // std::string path = "../../data/Dataset_1/D2/";
    // NoRotPipeline pipeline;
    // Dataset dataset(path);
    // print_metrics(pipeline, dataset);

    //TEST all datasets
    // StdPipeline pipeline(true, 8);
    // print_metrics_all(pipeline);
}