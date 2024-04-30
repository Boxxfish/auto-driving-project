#include <Eigen/Geometry> // Include this for additional Eigen functionalities

#include "visualizer.h"
#include "dataset.h"
#include "frame.h"
#include "pipeline.h"


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;



int main (int argc, char* argv[])
{

    StdPipeline pipeline();


    std::string path = "../../data/Dataset_1/D1/";
    Dataset dataset1_d1(path);

    std::cout << "Dataset Loaded from " << path << std::endl;

    Frame f1 = dataset1_d1.getFrame(1);
    std::cout << "Frame Loaded " << path << std::endl;

    dataset1_d1.c_poses_corrected[1] = pipeline.guess_v_pose(f1);

    PointCloudT::Ptr cloud_c_new (new PointCloudT); 
    pcl::transformPointCloud (*f1.cloud_c, *cloud_c_new, dataset1_d1.c_poses_corrected[1]);

    create_visualizer(std::string("Demo Visualizer"), f1.cloud_i, f1.cloud_c, cloud_c_new);




    



}