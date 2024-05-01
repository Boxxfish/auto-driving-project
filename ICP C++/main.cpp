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

    SimplePipeline pipeline(dataset1_d1);
    pipeline.run();


    //METRICS


    double easy_total_rte = 0.0;
    double hard_total_rte = 0.0;
    double easy_total_rre = 0.0;
    double hard_total_rre = 0.0;
    int easy_total = 0;
    int hard_total = 0;
    int easy_success = 0;
    int hard_success = 0;
    double easy_time = 0.0;
    double hard_time = 0.0;
    double SUCCESS_RTE = 2.0;

    for (int i = 0; i < pipeline.dataset.c_poses.size()-1; i++){
        double rre = compute_rre(pipeline.dataset.c_poses_corrected[i], pipeline.dataset.c_poses[i]);
        double rte = compute_rte(pipeline.dataset.c_poses_corrected[i], pipeline.dataset.c_poses[i]);
        if (std::find(easy_idxs.begin(), easy_idxs.end(), i) != easy_idxs.end()){
            easy_total_rte += rte;
            easy_total_rre += rre;
            if (rte <= SUCCESS_RTE)
            {
                easy_success += 1;
            }
            easy_time += pipeline.dataset.computation_time_list[i];
            easy_total += 1;
        }
        else
        {
            hard_total_rte += rte;
            hard_total_rre += rre;
            if (rte <= SUCCESS_RTE)
            {
                hard_success += 1;
            }
            hard_time += pipeline.dataset.computation_time_list[i];
            hard_total += 1;
        }

    }

    std::cout << "Avg. RTE (Easy): " << (float)easy_total_rte / (float)easy_total << std::endl;
    std::cout << "Avg. RRE (Easy): " << (float)easy_total_rre / (float)easy_total << std::endl;
    std::cout << "Success Rate (Easy): " << (float)easy_success / (float)easy_total << std::endl;
    std::cout << "Avg. Time (Easy): " << (float)easy_time / (float)easy_total << std::endl;
    std::cout << "Avg. RTE (Hard): " << (float)hard_total_rte / (float)hard_total << std::endl;
    std::cout << "Avg. RRE (Hard): " << (float)hard_total_rre / (float)hard_total << std::endl;
    std::cout << "Success Rate (Hard): " << (float)hard_success / (float)hard_total << std::endl;
    std::cout << "Avg. Time (Hard): " << (float)hard_time / (float)hard_total << std::endl;


    // //vizulization stuff

    // PointCloudT::Ptr cloud_c_new (new PointCloudT); 
    // std::cout << "original pose c: \n" << f1.pose_c << std::endl;
    // Eigen::Matrix4d t = f1.pose_i;
    // std::cout << "pose i: \n" << f1.pose_i << std::endl;
    // pcl::transformPointCloud (*f1.cloud_c, *cloud_c_new, dataset1_d1->c_poses_corrected[1]);
    // create_visualizer(std::string("Demo Visualizer"), f1.cloud_i, f1.cloud_c, cloud_c_new); 
}