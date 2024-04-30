

#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <optional>
#include "pipeline.h"
#include <pcl/filters/passthrough.h>
#include "ground_registration.h"



/// Our proposed pipeline.
/// Aligns 2 axes with the ground plane, then applies ground removal and performs ICP alignment.
/// Prior guesses are used as a starting point for future guesses.

Eigen::Matrix4d StdPipeline::guess_v_pose(Frame frame)
{
    // TODO: Set up the pipeline here.

    //fix infrastructure point cloud
    PointCloudT::Ptr cloud_c_new (new PointCloudT); 
    pcl::transformPointCloud (*frame.cloud_i, *frame.cloud_i, frame.pose_i);
    //add noise
    Eigen::Matrix4d pose = add_noise_xyz(frame.pose_c, 3);
    //ground align
    std::pair<Eigen::Vector3f, Eigen::Vector3f> planes = ground_plane(frame.cloud_c);
    std::cout << "Unit Normal Vector: \n" << planes.first << std::endl;
    std::cout << "Unit Direction Vector: \n" << planes.second << std::endl;

    //ground removal

    //icp
    std::cout << "end of pipeline" << std::endl;
    return pose;
}

Eigen::Matrix4d Pipeline::add_noise_xyz(const Eigen::Matrix4d& src,double stddev){

        std::cout << "add noise " << std::endl;
        std::default_random_engine generator;
        std::normal_distribution<double> dist(0.0, stddev);
        // Modify only x and y coords
        double x = dist(generator);
        double y = dist(generator);
        double z = dist(generator);

        std::cout << "x:  \n"<< x << std::endl;
        std::cout << "y:  \n"<< y << std::endl;
        std::cout << "z:  \n"<< z << std::endl;

        Eigen::Matrix4d new_pose;
        new_pose = src;
        std::cout << "new pose:  \n"<< new_pose << std::endl;

        new_pose(0,3) += x;
        new_pose(1,3) += y;
        new_pose(2,3) += z;
        std::cout << "new pose:  \n"<< new_pose << std::endl;


        return new_pose;
}

// not done
Eigen::Matrix4d Pipeline::add_noise_rot(const Eigen::Matrix4d &src, double stddev){
        std::default_random_engine generator;
        std::normal_distribution<double> dist(0.0, stddev);

        Eigen::Matrix3d rot;
        rot(0,0) = src(0,0);
        rot(0,1) = src(0,1);
        rot(0,2) = src(0,2);

        rot(1,0) = src(1,0);
        rot(1,1) = src(1,1);
        rot(1,2) = src(1,2);

        rot(2,0) = src(2,0);
        rot(2,1) = src(2,1);
        rot(2,2) = src(2,2);    

        Eigen::Quaterniond q_current(rot);


        Eigen::Matrix4d new_pose;
        // new_pose = *src;
        // new_pose(0,4) = x;
        // new_pose(0,4) = y;
        // new_pose(0,4) = z;

        return new_pose;
}


/// Given a source and target point cloud, returns a matrix that aligns the source to the target.
/// If alignment has failed, returns nullopt.
std::optional<Eigen::Matrix4d> Pipeline::align_icp(PointCloudT::Ptr src, PointCloudT::Ptr target, int iters) 
{
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaximumIterations(iters);
    icp.setInputTarget(target);
    icp.setInputSource(src);
    PointCloudT::Ptr cloud_final(new PointCloudT);
    icp.align(*cloud_final);
    return icp.hasConverged() ? std::optional{Eigen::Matrix4d(icp.getFinalTransformation().cast<double>())} : std::nullopt;
}

/// Given a point cloud, returns a new point cloud with ground points removed.
PointCloudT::Ptr Pipeline::remove_ground(PointCloudT::Ptr src) 
{
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(src);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-0.3, 0.3);
    pass.setNegative(true);
    pass.filter(*src);
    return src;
}

/// Given a point cloud, returns a vector indicating the "up" direction of the ground plane.
std::pair<Eigen::Vector3f, Eigen::Vector3f> Pipeline::ground_plane(PointCloudT::Ptr src)
{
    
    return getVectors(src);
}

