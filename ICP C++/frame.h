#pragma once
/*
 * frame.h
 */
#ifndef FRAME_H_
#define FRAME_H_

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include "dataset.h"
#include <Eigen/Geometry> // Include this for additional Eigen functionalities

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
class Dataset;
class Frame{
    private:
    public:
        PointCloudT::Ptr cloud_i;
        PointCloudT::Ptr cloud_c;
        Eigen::Matrix4d pose_i;
        Eigen::Matrix4d pose_c;
        Frame(int frame_num, Dataset* dataset);
        Frame(){}
};

#endif