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
#include <tuple>

#include "visualizer.h"

Eigen::Vector3f getDirectionVector(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*cloud, centroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Vector3f eigValues = eigensolver.eigenvalues();
    Eigen::Matrix3f eigVectors = eigensolver.eigenvectors();
    return eigVectors.col(2);  // Assuming the largest eigenvalue is last
}

std::tuple<Eigen::Vector3f, Eigen::Vector3f,pcl::PointCloud<pcl::PointXYZ>::Ptr> getVectors(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool remove_ground) {
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.3);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (remove_ground == true){
        // Extract non-inliers (points NOT on the plane)
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(true); // Extract non-plane points
        extract.filter(*filtered_cloud);
    }
    create_visualizer(std::string("ground reg Visualizer"), filtered_cloud, filtered_cloud, filtered_cloud); 



    Eigen::Vector3f unit_normal_vector(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    unit_normal_vector.normalize();

    pcl::PointCloud<pcl::PointXYZ>::Ptr groundCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.filter(*groundCloud);

    Eigen::Vector3f unit_direction_vector = getDirectionVector(groundCloud);

    return {unit_normal_vector, unit_direction_vector,filtered_cloud};
}

// Eigen::Matrix4f performRotationAlignment(const pcl::PointCloud<pcl::PointXYZ>::Ptr& infraCloud,
//                                          const pcl::PointCloud<pcl::PointXYZ>::Ptr& vehicleCloud) {
//     auto infraVectors = getVectors(infraCloud);
//     auto vehicleVectors = getVectors(vehicleCloud);

//     Eigen::Matrix4f rotationMatrix;
//     pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//     icp.setInputSource(infraCloud);
//     icp.setInputTarget(vehicleCloud);
//     pcl::PointCloud<pcl::PointXYZ> Final;
//     icp.align(Final);
//     rotationMatrix = icp.getFinalTransformation();

//     std::cout << "Infra vectors are:\n" << infraVectors.first.transpose() << std::endl;
//     std::cout << "Rotation Matrix is:\n" << rotationMatrix << std::endl;

//     return rotationMatrix;
// }

// void generateTransformedPCD(const pcl::PointCloud<pcl::PointXYZ>::Ptr& vehicleCloud,
//                             const pcl::PointCloud<pcl::PointXYZ>::Ptr& infraCloud,
//                             const Eigen::Matrix4f& rotationMatrix) {
//     pcl::PointCloud<pcl::PointXYZ> transformedInfraCloud;
//     pcl::transformPointCloud(*infraCloud, transformedInfraCloud, rotationMatrix);

//     pcl::PointCloud<pcl::PointXYZ> combinedCloud;
//     combinedCloud += *vehicleCloud;
//     combinedCloud += transformedInfraCloud;

//     pcl::io::savePCDFileASCII("combined_cloud.pcd", combinedCloud);
//     std::cout << "Saved combined cloud." << std::endl;
// }

// int test_ground_registration() {
//     pcl::PointCloud<pcl::PointXYZ>::Ptr vehicleCloud(new pcl::PointCloud<pcl::PointXYZ>());
//     pcl::PointCloud<pcl::PointXYZ>::Ptr infraCloud(new pcl::PointCloud<pcl::PointXYZ>());

//     if (pcl::io::loadPCDFile<pcl::PointXYZ>("vehicle.pcd", *vehicleCloud) == -1 ||
//         pcl::io::loadPCDFile<pcl::PointXYZ>("infrastructure.pcd", *infraCloud) == -1) {
//         PCL_ERROR("Couldn't read file\n");
//         return -1;
//     }

//     auto rotationMatrix = performRotationAlignment(infraCloud, vehicleCloud);
//     generateTransformedPCD(vehicleCloud, infraCloud, rotationMatrix);

//     return 0;
// }
