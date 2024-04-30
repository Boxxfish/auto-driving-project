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
        PointCloudT::Ptr remove_ground_basic(PointCloudT::Ptr src);

        /// create rotation matrix given vectors from gound plane analysis
        Eigen::Matrix3d create_rot_matrix(std::pair<Eigen::Vector3f, Eigen::Vector3f> vectors);

        Eigen::Matrix4d add_noise_xyz(const Eigen::Matrix4d &src,double stddev);

    public:
        /// Given a frame, returns where it thinks the vehicle is.
        /// This method is run on every frame in order.
        virtual Eigen::Matrix4d guess_v_pose(Frame &frame) = 0;
        /// Given a source and target point cloud, returns a matrix that aligns the source to the target.
        /// If alignment has failed, returns nullopt.
};

/// Our proposed pipeline.
/// Aligns 2 axes with the ground plane, then applies ground removal and performs ICP alignment.
/// Prior guesses are used as a starting point for future guesses.
class StdPipeline : public Pipeline
{
    public:
        Eigen::Matrix4d guess_v_pose(Frame &frame);
};



#endif