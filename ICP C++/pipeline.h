#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

// TODO: Replace this with our actual dataset stuff.
class Frame
{

}

class Pipeline
{
public:
    /// Given a frame, returns where it thinks the vehicle is.
    /// This method is run on every frame in order.
    virtual auto guess_v_pose(const Frame &frame) -> Eigen::Matrix4d;
}

/// Our proposed pipeline.
/// Aligns 2 axes with the ground plane, then applies ground removal and performs ICP alignment.
/// Prior guesses are used as a starting point for future guesses.
class StdPipeline : public Pipeline
{
public:
    StdPipeline() {}

    auto guess_v_pose(const Frame &frame) -> Eigen::Matrix4
    {
        // TODO: Set up the pipeline here.
    }
    std::optional<Eigen::Matrix4d> last_guess;
}

/// Given a source and target point cloud, returns a matrix that aligns the source to the target.
/// If alignment has failed, returns nullopt.
auto
align_icp(const PointCloudTargetConstPtr &src, const PointCloudTargetConstPtr &target) -> std::optional<Eigen::Matrix4d>
{
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaximumIterations(iterations);
    icp.setInputTarget(target);
    icp.setInputSource(src);
    PointCloudT::Ptr cloud_final(new PointCloudT);
    icp.align(*cloud_final);
    return icp.hasConverged() ? Eigen::Matrix4d(icp.getFinalTransformation().cast<double>()) : std::nullopt;
}

/// Given a point cloud, returns a new point cloud with ground points removed.
auto remove_ground(const PointCloudTargetConstPtr &src) -> const PointCloudTargetConstPtr&
{
    pcl::PassThrough<pcl::PointXYZ> pass_c;
    pass_c.setInputCloud(src);
    pass_c.setFilterFieldName("z");
    pass_c.setFilterLimits(-0.3, 0.3);
    pass_c.setNegative(true);
    pass_c.filter(*cloud_c);
    return c;
}

/// Given a point cloud, returns a vector indicating the "up" direction of the ground plane.
auto ground_plane(const PointCloudTargetConstPtr &src) -> Eigen::Vector3d
{
}