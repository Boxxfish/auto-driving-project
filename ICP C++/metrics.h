#ifndef METRICS_H_
#define METRICS_H_

#include <Eigen/Geometry>

// Function to calculate and return RRE (degrees).
auto compute_rre(const Eigen::Matrix4d &current_transformation, const Eigen::Matrix4d &ground_truth_transformation) -> double
{
    // Extract rotation matrices
    Eigen::Matrix3d R_current = current_transformation.block<3, 3>(0, 0);
    Eigen::Matrix3d R_ground_truth = ground_truth_transformation.block<3, 3>(0, 0);

    // Calculate RRE as the angle between the two rotation matrices
    Eigen::Quaterniond q_current(R_current), q_ground_truth(R_ground_truth);
    double RRE = q_current.angularDistance(q_ground_truth) * (180.0 / M_PI);

    return RRE;
}

// Function to calculate and return RTE (meters).
auto compute_rte(const Eigen::Matrix4d &current_transformation, const Eigen::Matrix4d &ground_truth_transformation) -> double
{
    // Extract translation vectors
    Eigen::Vector3d t_current = current_transformation.block<3, 1>(0, 3);
    Eigen::Vector3d t_ground_truth = ground_truth_transformation.block<3, 1>(0, 3);

    // Calculate RTE as Euclidean distance between the two translation vectors
    double RTE = (t_current - t_ground_truth).norm();
    return RTE;
}

#endif