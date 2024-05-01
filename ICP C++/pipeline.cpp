

#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <optional>
#include "pipeline.h"
#include <pcl/filters/passthrough.h>
#include "ground_registration.h"
#include <tuple>
#include "visualizer.h"
#include "metrics.h"

/// Our proposed pipeline.
/// Aligns 2 axes with the ground plane, then applies ground removal and performs ICP alignment.
/// Prior guesses are used as a starting point for future guesses.

Eigen::Matrix4d StdPipeline::guess_v_pose(const Frame &frame, const Eigen::Matrix4d &i_pose)
{

    pcl::transformPointCloud(*frame.cloud_i, *frame.cloud_i, i_pose);
    PointCloudT::Ptr i_temp(new PointCloudT);
    *i_temp = *frame.cloud_i;
    i_temp = remove_ground_basic(i_temp);

    Eigen::Matrix4d pose = get_gps_location(frame.pose_c, 3);

    PointCloudT::Ptr c_temp(new PointCloudT);
    *c_temp = *frame.cloud_c;

    // ground align
    bool remove_ground = true;
    // gets vectors and removes ground from car point cloud
    std::tuple<Eigen::Vector3f, Eigen::Vector3f, PointCloudT::Ptr> vectors = getVectors(c_temp, remove_ground);
    c_temp = std::get<2>(vectors);

    // angle guess
    auto rot = get_best_rotation(frame, i_temp);

    pose(0, 0) = rot(0, 0);
    pose(0, 1) = rot(0, 1);
    pose(0, 2) = rot(0, 2);

    pose(1, 0) = rot(1, 0);
    pose(1, 1) = rot(1, 1);
    pose(1, 2) = rot(1, 2);

    pose(2, 0) = rot(2, 0);
    pose(2, 1) = rot(2, 1);
    pose(2, 2) = rot(2, 2);

    pcl::transformPointCloud(*frame.cloud_c, *c_temp, pose);

    // icp
    Eigen::Matrix4d icp_pose = align_icp(c_temp, i_temp, 50);

    // std::cout << "end of pipeline" << std::endl;
    return icp_pose * pose;
}

// keeps rotation estiates from ground truth
Eigen::Matrix4d SimplePipeline::guess_v_pose(const Frame &frame, const Eigen::Matrix4d &i_pose)
{

    // fix infrastructure point cloud
    pcl::transformPointCloud(*frame.cloud_i, *frame.cloud_i, i_pose);
    PointCloudT::Ptr i_temp(new PointCloudT);
    *i_temp = *frame.cloud_i;
    i_temp = remove_ground_basic(i_temp);

    // add noise
    //  Eigen::Matrix4d pose;
    //  pose(3,3) = 1;
    Eigen::Matrix4d pose = get_gps_location(frame.pose_c, 3);
    // std::cout << "Noise Added: \n" << pose << std::endl;

    pose(0, 0) = frame.pose_c(0, 0);
    pose(0, 1) = frame.pose_c(0, 1);
    pose(0, 2) = frame.pose_c(0, 2);

    pose(1, 0) = frame.pose_c(1, 0);
    pose(1, 1) = frame.pose_c(1, 1);
    pose(1, 2) = frame.pose_c(1, 2);

    pose(2, 0) = frame.pose_c(2, 0);
    pose(2, 1) = frame.pose_c(2, 1);
    pose(2, 2) = frame.pose_c(2, 2);

    PointCloudT::Ptr c_temp(new PointCloudT);
    pcl::transformPointCloud(*frame.cloud_c, *c_temp, pose);

    // ground align
    bool remove_ground = true;
    // gets vectors and removes ground from car point cloud
    std::tuple<Eigen::Vector3f, Eigen::Vector3f, pcl::PointCloud<pcl::PointXYZ>::Ptr> vectors = getVectors(c_temp, remove_ground);
    c_temp = std::get<2>(vectors);

    // icp
    Eigen::Matrix4d icp_pose = align_icp(c_temp, i_temp, 50);

    // std::cout << "end of pipeline" << std::endl;
    return icp_pose * pose;
}

Eigen::Matrix4d InterpolationPipeline::guess_v_pose(const Frame &frame, const Eigen::Matrix4d &i_pose)
{
    // fix infrastructure point cloud
    pcl::transformPointCloud(*frame.cloud_i, *frame.cloud_i, i_pose);
    PointCloudT::Ptr i_temp(new PointCloudT);
    *i_temp = *frame.cloud_i;
    i_temp = remove_ground_basic(i_temp);

    // add noise
    //  Eigen::Matrix4d pose;
    //  pose(3,3) = 1;
    Eigen::Matrix4d pose = get_gps_location(frame.pose_c, 3);
    // std::cout << "Noise Added: \n" << pose << std::endl;

    pose(0, 0) = frame.pose_c(0, 0);
    pose(0, 1) = frame.pose_c(0, 1);
    pose(0, 2) = frame.pose_c(0, 2);

    pose(1, 0) = frame.pose_c(1, 0);
    pose(1, 1) = frame.pose_c(1, 1);
    pose(1, 2) = frame.pose_c(1, 2);

    pose(2, 0) = frame.pose_c(2, 0);
    pose(2, 1) = frame.pose_c(2, 1);
    pose(2, 2) = frame.pose_c(2, 2);

    PointCloudT::Ptr c_temp(new PointCloudT);
    pcl::transformPointCloud(*frame.cloud_c, *c_temp, pose);

    // ground align
    bool remove_ground = true;
    // gets vectors and removes ground from car point cloud
    std::tuple<Eigen::Vector3f, Eigen::Vector3f, pcl::PointCloud<pcl::PointXYZ>::Ptr> vectors = getVectors(c_temp, remove_ground);
    c_temp = std::get<2>(vectors);

    // icp
    Eigen::Matrix4d icp_pose = align_icp(c_temp, i_temp, 50);

    // std::cout << "end of pipeline" << std::endl;
    return icp_pose * pose;
}

// Eigen::Matrix4d location_interpolation(const Frame &f1, Eigen::Matrix4d translation, Frame &fn)
// {
//     // fix infrastructure point cloud
//     pcl::transformPointCloud(*fn.cloud_i, *fn.cloud_i, fn.pose_i);
//     fn.cloud_i = remove_ground_basic(fn.cloud_i);

//     Eigen::Matrix4d pose = get_gps_location(fn.pose_c, 3);
//     pcl::transformPointCloud(*fn.cloud_c, *fn.cloud_c, pose);

//     // ground align
//     bool remove_ground = true;
//     // gets vectors and removes ground from car point cloud
//     std::tuple<Eigen::Vector3f, Eigen::Vector3f, pcl::PointCloud<pcl::PointXYZ>::Ptr> vectors = getVectors(fn.cloud_c, remove_ground);
//     fn.cloud_c = std::get<2>(vectors);

//     PointCloudT::Ptr cloud_c1(new PointCloudT);
//     pcl::transformPointCloud(*fn.cloud_c, *cloud_c1, pose);

//     Eigen::Matrix4d icp_pose = align_icp(fn.cloud_c, cloud_c1, 150);
//     pose = pose * icp_pose;

//     return pose;
// }

Eigen::Matrix4d get_gps_location(const Eigen::Matrix4d &src, double stddev)
{
    std::default_random_engine generator;
    std::normal_distribution<double> dist(0.0, stddev);
    // Modify only x and y coords
    double x = dist(generator);
    double y = dist(generator);
    double z = dist(generator);

    Eigen::Matrix4d new_pose;
    new_pose << 1.0, 0.0, 0.0, src(0, 3) + x,
        0.0, 1.0, 0.0, src(1, 3) + y,
        0.0, 0.0, 1.0, src(2, 3) + z,
        0.0, 0.0, 0.0, 1.0;
    return new_pose;
}

/// Given a source and target point cloud, returns a matrix that aligns the source to the target.
/// If alignment has failed, returns nullopt.
Eigen::Matrix4d align_icp(PointCloudT::Ptr src, PointCloudT::Ptr target, int iters, int max_corresp_dist)
{
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaximumIterations(iters);
    icp.setInputTarget(target);
    icp.setInputSource(src);
    icp.setMaxCorrespondenceDistance(max_corresp_dist);
    PointCloudT::Ptr cloud_final(new PointCloudT);
    icp.align(*cloud_final);
    // create_visualizer(std::string("Demo Visualizer"), target, src, cloud_final);
    Eigen::Matrix4d transform = Eigen::Matrix4d(icp.getFinalTransformation().cast<double>());
    return transform;
}

/// Given a point cloud, returns a new point cloud with ground points removed.
PointCloudT::Ptr remove_ground_basic(PointCloudT::Ptr src)
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
Eigen::Matrix3d create_rot_matrix(Eigen::Vector3f z, Eigen::Vector3f y)
{
    Eigen::Vector3f x = z.cross(y);

    Eigen::Matrix3d rot;
    rot << x.dot(x), y.dot(x), z.dot(x),
        x.dot(y), y.dot(y), z.dot(y),
        x.dot(z), y.dot(z), z.dot(z);

    return rot;
}



Eigen::Matrix4d get_best_rotation(Frame frame, PointCloudT::Ptr target){
    std::vector<double> scores;
    double degree = 0;

    Eigen::Matrix4d pose;
    PointCloudT::Ptr c_0 (new PointCloudT); 
    *c_0 = *frame.cloud_c;
    scores.push_back(get_icp_score(c_0, target));

    PointCloudT::Ptr c_45 (new PointCloudT); 
    degree = 45;
    pose = make_custom_rot_matrix(degree);
    pcl::transformPointCloud(*c_0, *c_45, pose);
    scores.push_back(get_icp_score(c_45, target));

    PointCloudT::Ptr c_90 (new PointCloudT);
    degree = 90; 
    pose = make_custom_rot_matrix(degree);
    pcl::transformPointCloud(*c_0, *c_90, pose);
    scores.push_back(get_icp_score(c_90, target));

    PointCloudT::Ptr c_135 (new PointCloudT); 
    degree = 135;
    pose = make_custom_rot_matrix(degree);
    pcl::transformPointCloud(*c_0, *c_135, pose);
    scores.push_back(get_icp_score(c_135, target));

    PointCloudT::Ptr c_180 (new PointCloudT); 
    degree = 180;
    pose = make_custom_rot_matrix(degree);
    pcl::transformPointCloud(*c_0, *c_180, pose);
    scores.push_back(get_icp_score(c_180, target));

    PointCloudT::Ptr c_225 (new PointCloudT); 
    degree = 225;
    pose = make_custom_rot_matrix(degree);
    pcl::transformPointCloud(*c_0, *c_225, pose);
    scores.push_back(get_icp_score(c_225, target));

    PointCloudT::Ptr c_270 (new PointCloudT); 
    degree = 270;
    pose = make_custom_rot_matrix(degree);
    pcl::transformPointCloud(*c_0, *c_270, pose);
    scores.push_back(get_icp_score(c_270, target));

    PointCloudT::Ptr c_315 (new PointCloudT); 
    degree = 315;
    pose = make_custom_rot_matrix(degree);
    pcl::transformPointCloud(*c_0, *c_315, pose);
    scores.push_back(get_icp_score(c_315, target));

    int i = min_element(scores.begin(),scores.end()) - scores.begin();
    degree = i*45;
    return make_custom_rot_matrix(degree);
}

double get_icp_score(PointCloudT::Ptr src, PointCloudT::Ptr target){
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaximumIterations(10);
    icp.setInputTarget(target);
    icp.setInputSource(src);
    icp.setMaxCorrespondenceDistance(3);
    PointCloudT::Ptr cloud_final(new PointCloudT);
    icp.align(*cloud_final);
    return icp.getFitnessScore();
}

Eigen::Matrix4d make_custom_rot_matrix(double degrees){
    double theta = degrees * (M_PI / 180.0); 

    // Create a rotation matrix around the x-axis
    Eigen::Matrix4d rotation_matrix;
    rotation_matrix << 1, 0, 0, 1,
                       0, cos(theta), -sin(theta), 1,
                       0, sin(theta), cos(theta), 1,
                       0, 0, 0, 1;
    return rotation_matrix;    
}
