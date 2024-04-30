#ifndef GROUND_REGISTRATION_H_
#define GROUND_REGISTRATION_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <iostream>
#include <fstream>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>

Eigen::Vector3f getDirectionVector(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

std::tuple<Eigen::Vector3f, Eigen::Vector3f, pcl::PointCloud<pcl::PointXYZ>::Ptr> getVectors(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool remove_ground);

Eigen::Matrix4f performRotationAlignment(const pcl::PointCloud<pcl::PointXYZ>::Ptr& infraCloud,
                                         const pcl::PointCloud<pcl::PointXYZ>::Ptr& vehicleCloud);

void generateTransformedPCD(const pcl::PointCloud<pcl::PointXYZ>::Ptr& vehicleCloud,
                            const pcl::PointCloud<pcl::PointXYZ>::Ptr& infraCloud,
                            const Eigen::Matrix4f& rotationMatrix);

/// Tests ground registration.
/// Wrap this in your `main()` function to perform the test. 
int test_ground_registration();

#endif