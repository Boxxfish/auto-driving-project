

#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <optional>
#include "pipeline.h"
#include <pcl/filters/passthrough.h>
#include "ground_registration.h"
#include <tuple>
#include "visualizer.h"
#include <pcl/console/time.h> // TicToc
#include "metrics.h"

/// Our proposed pipeline.
/// Aligns 2 axes with the ground plane, then applies ground removal and performs ICP alignment.
/// Prior guesses are used as a starting point for future guesses.

Eigen::Matrix4d StdPipeline::guess_v_pose(const Frame &frame, const Eigen::Matrix4d &i_pose)
{

    pcl::transformPointCloud(*frame.cloud_i, *frame.cloud_i, frame.pose_i);
    PointCloudT::Ptr i_temp(new PointCloudT);
    *i_temp = *frame.cloud_i;
    i_temp = remove_ground_basic(i_temp);

    // add noise
    //  Eigen::Matrix4d pose;
    //  pose(3,3) = 1;
    Eigen::Matrix4d pose = get_gps_location(frame.pose_c, 3);
    // std::cout << "Noise Added: \n" << pose << std::endl;

    PointCloudT::Ptr c_temp(new PointCloudT);
    *c_temp = *frame.cloud_c;

    // ground align
    bool remove_ground = true;
    // gets vectors and removes ground from car point cloud
    std::tuple<Eigen::Vector3f, Eigen::Vector3f, pcl::PointCloud<pcl::PointXYZ>::Ptr> vectors = getVectors(c_temp, remove_ground);
    c_temp = std::get<2>(vectors);

    // std::cout << "pose i: \n" << frame.pose_i << std::endl;
    // std::cout << "pose c: \n" << pose << std::endl;

    // Eigen::Vector3f diff;
    // diff(0) =  ((float)frame.pose_i(0,3) - (float)pose(0,3));
    // diff(1) = ((float)frame.pose_i(1,3) - (float)pose(1,3));
    // diff(2) = 0;
    // std::cout << "diff: \n" << diff << std::endl;

    // diff.normalize();
    // std::cout << "diff: \n" << diff << std::endl;

    // Eigen::Matrix3d rot = create_rot_matrix(diff,get<1>(vectors));
    // pose(0,0) = rot(0,0);
    // pose(0,1) = rot(0,1);
    // pose(0,2) = rot(0,2);

    // pose(1,0) = rot(1,0);
    // pose(1,1) = rot(1,1);
    // pose(1,2) = rot(1,2);

    // pose(2,0) = rot(2,0);
    // pose(2,1) = rot(2,1);
    // pose(2,2) = rot(2,2);

    // std::cout << "Rotation Guess Added: \n" << pose << std::endl;

    pcl::transformPointCloud(*frame.cloud_c, *c_temp, pose);

    // icp
    Eigen::Matrix4d icp_pose = Pipeline::align_icp(c_temp, i_temp, 50);

    // std::cout << "end of pipeline" << std::endl;
    return icp_pose * pose;
}

// keeps rotation estiates from ground truth
Eigen::Matrix4d SimplePipeline::guess_v_pose(Frame &frame)
{

    // fix infrastructure point cloud
    pcl::transformPointCloud(*frame.cloud_i, *frame.cloud_i, frame.pose_i);
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
    Eigen::Matrix4d icp_pose = Pipeline::align_icp(c_temp, i_temp, 50);

    // std::cout << "end of pipeline" << std::endl;
    return icp_pose * pose;
}

Eigen::Matrix4d InterpolationPipeline::guess_v_pose(Frame &frame)
{
    // fix infrastructure point cloud
    pcl::transformPointCloud(*frame.cloud_i, *frame.cloud_i, frame.pose_i);
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
    Eigen::Matrix4d icp_pose = Pipeline::align_icp(c_temp, i_temp, 50);

    // std::cout << "end of pipeline" << std::endl;
    return icp_pose * pose;
}

Eigen::Matrix4d Pipeline::location_interpolation(Frame &f1, Eigen::Matrix4d translation, Frame &fn)
{
    // fix infrastructure point cloud
    pcl::transformPointCloud(*fn.cloud_i, *fn.cloud_i, fn.pose_i);
    fn.cloud_i = remove_ground_basic(fn.cloud_i);

    Eigen::Matrix4d pose = get_gps_location(fn.pose_c, 3);
    pcl::transformPointCloud(*fn.cloud_c, *fn.cloud_c, pose);

    // ground align
    bool remove_ground = true;
    // gets vectors and removes ground from car point cloud
    std::tuple<Eigen::Vector3f, Eigen::Vector3f, pcl::PointCloud<pcl::PointXYZ>::Ptr> vectors = getVectors(fn.cloud_c, remove_ground);
    fn.cloud_c = std::get<2>(vectors);

    PointCloudT::Ptr cloud_c1(new PointCloudT);
    pcl::transformPointCloud(*fn.cloud_c, *cloud_c1, pose);

    Eigen::Matrix4d icp_pose = align_icp(fn.cloud_c, cloud_c1, 150);
    pose = pose * icp_pose;

    return pose;
}

Eigen::Matrix4d Pipeline::get_gps_location(const Eigen::Matrix4d &src, double stddev)
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
    create_visualizer(std::string("Demo Visualizer"), target, src, cloud_final);
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

// /// Given a point cloud, returns a new point cloud with ground points removed.
// PointCloudT::Ptr Pipeline::remove_ground_given_plane(PointCloudT::Ptr src, Eigen::Vector3f plane_normal)
// {
//     float distanceThreshold = 0.3f; // Adjust as needed

//     // Create the filter object
//     pcl::PassThrough<pcl::PointXYZ> pass;
//     pass.setInputCloud(src);
//     pass.setFilterFieldName("z"); // Assuming z is the vertical axis in your point cloud
//     pass.setFilterLimits(-std::numeric_limits<float>::max(), std::numeric_limits<float>::max()); // Keep all points initially

//     // Set the filter condition based on the distance to the plane
//     Eigen::Vector3f planePoint(0, 0, 0); // Assuming the plane passes through the origin
//     float distanceThresholdSquared = distanceThreshold * distanceThreshold; // Squared distance threshold for efficiency
//     pass.setFilterFunction([&plane_normal, &planePoint, distanceThresholdSquared](const pcl::PointXYZ& pt) {
//         Eigen::Vector3f point(pt.x, pt.y, pt.z);
//         Eigen::Vector3f vec = point - planePoint;
//         float distSquared = vec.squaredNorm();
//         float dotProduct = vec.dot(plane_normal);
//         return distSquared > distanceThresholdSquared || dotProduct > 0; // Keep points that are far from the plane or on the correct side
//     });

//     // Apply the filter
//     pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
//     pass.filter(*filteredCloud);

//     return

// }

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
