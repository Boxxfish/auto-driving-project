#include "frame.h"
#include <string>

#include <pcl/io/pcd_io.h>

Frame::Frame(int frame_num, Dataset* dataset){
    this->pose_i = dataset->i_pose;
    this->pose_c = dataset->c_poses[frame_num];
    std::cout << "Poses loaded from dataset." << std::endl;
    PointCloudT::Ptr cloud_i_temp (new PointCloudT);  
    PointCloudT::Ptr cloud_c_temp (new PointCloudT); 
    std::cout << "Poses loaded from dataset." << std::endl;

    this->cloud_i = cloud_i_temp;
    this->cloud_c = cloud_c_temp;
    std::cout << "Poses loaded from dataset." << std::endl;

    if (pcl::io::loadPCDFile((dataset->path)+"splits/"+std::to_string(frame_num)+"_i.pcd", *(this->cloud_i)) < 0)
    {
        PCL_ERROR ("Error loading cloud.\n");
        exit(-1);
    }

    if (pcl::io::loadPCDFile((dataset->path)+"splits/"+std::to_string(frame_num)+"_v.pcd", *(this->cloud_c)) < 0)
    {
        PCL_ERROR ("Error loading cloud.\n");
        exit(-1);
    }
    std::cout << "Point Clouds Loaded" << std::endl;

}