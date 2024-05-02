#pragma once

#ifndef PIPELINE_H_
#define PIPELINE_H_

#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <optional>
#include <random>
#include "dataset.h"

// TODO: Replace this with our actual dataset stuff.

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class Frame;
class Dataset;

class Pipeline
{
public:
    Pipeline() {}

    /// Given a frame, returns where it thinks the vehicle is.
    /// This method is run on every frame in order.
    virtual std::optional<Eigen::Matrix4d> guess_v_pose(const Frame &frame, const Eigen::Matrix4d &i_pose) = 0;
};

/// Our proposed pipeline.
/// Aligns 2 axes with the ground plane, then applies ground removal and performs ICP alignment.
/// Prior guesses are used as a starting point for future guesses.
class StdPipeline : public Pipeline
{
protected:
    Eigen::Matrix4d get_initial_T(const Frame &frame, const Eigen::Matrix4d &i_pose, bool remove_ground);
    Eigen::Matrix4d get_interpolation_T(const Frame &frame, const Eigen::Matrix4d &i_pose, bool remove_ground);

public:
    StdPipeline(bool remove_ground) : Pipeline() {
        this->remove_ground = remove_ground;
    }
    std::optional<Eigen::Matrix4d> T_previous;
    std::optional<Eigen::Vector3d> i_gps;
    int incoming_threshold = 200;
    int outgoing_threshold = 200;
    bool remove_ground;

    std::optional<Eigen::Matrix4d> guess_v_pose(const Frame &frame, const Eigen::Matrix4d &i_pose);

};

// only uses icp, uses ground truth rotation in initial guess
// still does ground removal
class SimplePipeline : public Pipeline
{
public:
    SimplePipeline() : Pipeline() {}
    std::optional<Eigen::Matrix4d> guess_v_pose(const Frame &frame, const Eigen::Matrix4d &i_pose);
};

// only uses icp, uses ground truth rotation in initial guess
// still does ground removal
class InterpolationPipeline : public Pipeline
{
public:
    InterpolationPipeline() : Pipeline() {}
    std::optional<Eigen::Matrix4d> guess_v_pose(const Frame &frame, const Eigen::Matrix4d &i_pose);
};

/// Given a source and target point cloud, returns a matrix that aligns the source to the target.
/// If alignment has failed, returns nullopt.
Eigen::Matrix4d align_icp(PointCloudT::Ptr src, PointCloudT::Ptr target, int iters, int max_corresp_dist = 3);

/// Given a point cloud, returns a new point cloud with ground points removed.
PointCloudT::Ptr remove_ground_basic(PointCloudT::Ptr src);

/// create rotation matrix given vectors from gound plane analysis
Eigen::Matrix3d create_rot_matrix(Eigen::Vector3f z, Eigen::Vector3f y);

Eigen::Matrix4d get_gps_location(const Eigen::Matrix4d &src, double stddev);

// Eigen::Matrix4d location_interpolation(Frame &f1, Eigen::Matrix4d translation, Frame &fn);
Eigen::Matrix4d get_best_rotation(PointCloudT::Ptr src, PointCloudT::Ptr target,Eigen::Matrix4d translate);
double get_icp_score(PointCloudT::Ptr src, PointCloudT::Ptr target);
Eigen::Matrix4d make_custom_rot_matrix(double degrees);

#endif