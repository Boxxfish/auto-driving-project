#pragma once
/*
 * dataset.h
 */
#ifndef DATASET_H_
#define DATASET_H_

#include <string>
#include <Eigen/Geometry> // Include this for additional Eigen functionalities
#include "frame.h"

class Frame;

/// A frame of data.
struct Frame
{
    /// Point cloud for infra.
    PointCloudT::Ptr cloud_i;
    /// Point cloud for vehicle.
    PointCloudT::Ptr cloud_c;
    /// Vehicle ground truth pose.
    Eigen::Matrix4d pose_c;
};

class Dataset
{
public:
    std::string path;
    /// Infra ground truth pose.
    Eigen::Matrix4d i_pose;
    /// Frames in our dataset, should be of length 300.
    std::vector<Frame>

    /// Loads poses and frames from a path.
    Dataset(const std::string &path);
};

/// Reads a list of poses from a file.
std::vector<Eigen::Matrix4d> read_poses(const std::string &path);

/// Loads a point cloud from a file.
PointCloudT::Ptr load_pc(const std::string &path);

#endif