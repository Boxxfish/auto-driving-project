#pragma once

#ifndef PIPELINE_H_
#define PIPELINE_H_

#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <optional>
#include <random>
#include "frame.h"

// TODO: Replace this with our actual dataset stuff.

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class Frame;

class Pipeline
{
    protected:
        std::optional<Eigen::Matrix4d> align_icp(PointCloudT::Ptr src, PointCloudT::Ptr target, int iters);

        /// Given a point cloud, returns a new point cloud with ground points removed.
        PointCloudT::Ptr remove_ground(PointCloudT::Ptr src);

        /// Given a point cloud, returns a vector indicating the "up" direction of the ground plane.
        Eigen::Vector3d ground_plane(PointCloudT::Ptr src);

        Eigen::Matrix4d add_noise_xyz(const Eigen::Matrix4d &src,double stddev);
        Eigen::Matrix4d add_noise_rot(const Eigen::Matrix4d &src,double stddev);

    public:
        /// Given a frame, returns where it thinks the vehicle is.
        /// This method is run on every frame in order.
        virtual Eigen::Matrix4d guess_v_pose(const Frame frame) = 0;
        /// Given a source and target point cloud, returns a matrix that aligns the source to the target.
        /// If alignment has failed, returns nullopt.
};

/// Our proposed pipeline.
/// Aligns 2 axes with the ground plane, then applies ground removal and performs ICP alignment.
/// Prior guesses are used as a starting point for future guesses.
class StdPipeline : public Pipeline
{
    public:
        Eigen::Matrix4d guess_v_pose(const Frame frame);
};



#endif