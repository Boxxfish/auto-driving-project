#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <optional>

// TODO: Replace this with our actual dataset stuff.
class Frame
{

};

class Pipeline
{
public:
    /// Given a frame, returns where it thinks the vehicle is.
    /// This method is run on every frame in order.
    virtual auto guess_v_pose(const Frame &frame) -> Eigen::Matrix4d;
};

/// Our proposed pipeline.
/// Aligns 2 axes with the ground plane, then applies ground removal and performs ICP alignment.
/// Prior guesses are used as a starting point for future guesses.
class StdPipeline : public Pipeline
{
public:
    StdPipeline() {}

    auto guess_v_pose(const Frame &frame) -> Eigen::Matrix4d
    {
        // TODO: Set up the pipeline here.
        return Eigen::Matrix4d();
    }
    std::optional<Eigen::Matrix4d> last_guess;
};

/// Given a source and target point cloud, returns a matrix that aligns the source to the target.
/// If alignment has failed, returns nullopt.
auto
align_icp(PointCloudT::Ptr src, PointCloudT::Ptr target, int iters) -> std::optional<Eigen::Matrix4d>
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
auto remove_ground(PointCloudT::Ptr src) -> PointCloudT::Ptr
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
auto ground_plane(PointCloudT::Ptr src) -> Eigen::Vector3d
{
    return Eigen::Vector3d();
}