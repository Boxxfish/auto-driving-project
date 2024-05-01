#pragma once

#ifndef PIPELINE_H_
#define PIPELINE_H_

#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <optional>
#include <random>
#include "frame.h"
#include "dataset.h"

// TODO: Replace this with our actual dataset stuff.

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class Frame;
class Dataset;

class Pipeline
{
    protected:
        Eigen::Matrix4d align_icp(PointCloudT::Ptr src, PointCloudT::Ptr target, int iters,int max_corresp_dist = 3);

        /// Given a point cloud, returns a new point cloud with ground points removed.
        PointCloudT::Ptr remove_ground_basic(PointCloudT::Ptr src);

        /// create rotation matrix given vectors from gound plane analysis
        Eigen::Matrix3d create_rot_matrix(Eigen::Vector3f z, Eigen::Vector3f y);

        Eigen::Matrix4d get_gps_location(const Eigen::Matrix4d &src,double stddev);

        Eigen::Matrix4d location_interpolation(Frame &f1, Eigen::Matrix4d translation, Frame &fn);

    public:
        Dataset dataset;
        Pipeline(Dataset& dataset); //constructor

        //runs on the whole dataset
        virtual void run() = 0;

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
        void run();
};

// only uses icp, uses ground truth rotation in initial guess
// still does ground removal
class SimplePipeline : public Pipeline
{
    public:
        Eigen::Matrix4d guess_v_pose(Frame &frame);
        void run();
};

// only uses icp, uses ground truth rotation in initial guess
// still does ground removal
class InterpolationPipeline : public Pipeline
{
    public:
        Eigen::Matrix4d guess_v_pose(Frame &frame);
        void run();
};

#endif