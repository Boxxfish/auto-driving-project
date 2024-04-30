

#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <optional>
#include "pipeline.h"
#include <pcl/filters/passthrough.h>
#include "ground_registration.h"
#include <tuple>
#include "visualizer.h"



/// Our proposed pipeline.
/// Aligns 2 axes with the ground plane, then applies ground removal and performs ICP alignment.
/// Prior guesses are used as a starting point for future guesses.

Eigen::Matrix4d StdPipeline::guess_v_pose(Frame &frame)
{
    // TODO: Set up the pipeline here.

    //fix infrastructure point cloud
    PointCloudT::Ptr cloud_c_new (new PointCloudT); 
    pcl::transformPointCloud (*frame.cloud_i, *frame.cloud_i, frame.pose_i);
    frame.cloud_i = remove_ground_basic(frame.cloud_i);

    //add noise
    Eigen::Matrix4d pose = add_noise_xyz(frame.pose_c, 3);
    std::cout << "Noise Added: \n" << pose << std::endl;
    //ground align
    bool remove_ground=true;
    std::tuple<Eigen::Vector3f, Eigen::Vector3f, pcl::PointCloud<pcl::PointXYZ>::Ptr> vectors = getVectors(frame.cloud_c, remove_ground);
    frame.cloud_c = get<2>(vectors);
    create_visualizer(std::string("ground reg Visualizer"), frame.cloud_c, frame.cloud_c, frame.cloud_c); 

    // Eigen::Matrix3d rot = create_rot_matrix(vectors);
    // pose(0,0) = rot(0,0);
    // pose(0,1) = rot(0,1);
    // pose(0,2) = rot(0,2);

    // pose(1,0) = rot(1,0);
    // pose(1,1) = rot(1,1);
    // pose(1,2) = rot(1,2);

    // pose(2,0) = rot(2,0);
    // pose(2,1) = rot(2,1);
    // pose(2,2) = rot(2,2);

    std::cout << "Rotation Guess Added: \n" << pose << std::endl;


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
PointCloudT::Ptr Pipeline::remove_ground_basic(PointCloudT::Ptr src) 
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
Eigen::Matrix3d Pipeline::create_rot_matrix(std::pair<Eigen::Vector3f, Eigen::Vector3f> vectors)
{
    Eigen::Vector3f z = vectors.first;
    Eigen::Vector3f y = vectors.second;
    std::cout << "Unit Normal Vector: \n" << z << std::endl;
    std::cout << "Unit Direction Vector: \n" << y << std::endl;

    Eigen::Vector3f x = z.cross(y);

    Eigen::Matrix3d rot;
    rot << x.dot(x), y.dot(x), z.dot(x),
    x.dot(y), y.dot(y), z.dot(y), 
    x.dot(z), y.dot(z), z.dot(z);

    return rot;
}

