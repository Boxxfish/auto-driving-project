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

class Dataset{
    private:
        void load_i_pose();
        void load_c_poses();
    public:
        std::string path;
        Eigen::Matrix4d i_pose;
        std::vector<Eigen::Matrix4d> c_poses;
        std::vector<Eigen::Matrix4d> c_poses_corrected;
        std::vector<double> computation_time_list;

        Dataset();
        Dataset(std::string& path); //load ups the poses
        Frame getFrame(int frame_name);
};

#endif