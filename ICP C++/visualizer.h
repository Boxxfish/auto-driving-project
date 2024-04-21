#pragma once
/*
 * visualizer.h
 */
#ifndef VISUALIZER_H_
#define VISUALIZER_H_
#include <string>
#include <pcl/point_types.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// Contents of the file go here
void create_visualizer(const std::string& name, PointCloudT::Ptr cloud_i, PointCloudT::Ptr cloud_c_original, PointCloudT::Ptr cloud_c_transformed);

#endif