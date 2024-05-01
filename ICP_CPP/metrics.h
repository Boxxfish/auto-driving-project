#pragma once
#ifndef METRICS_H_
#define METRICS_H_

#include <Eigen/Geometry>
#include "pipeline.h"

// Function to calculate and return RRE (degrees).
double compute_rre(const Eigen::Matrix4d &current_transformation, const Eigen::Matrix4d &ground_truth_transformation);

// Function to calculate and return RTE (meters).
double compute_rte(const Eigen::Matrix4d &current_transformation, const Eigen::Matrix4d &ground_truth_transformation);

/// Given a pipeline and a dataset, runs the pipeline on the dataset and collects metrics.
void print_metrics(Pipeline &pipeline, const Dataset& dataset);

#endif