#include <iostream>
#include <string>
#include <random>
#include <typeinfo>
#include <fstream>
#include <sstream>
#include <cmath> // for radians conversion
#include <ranges>
#include <algorithm>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h> // TicToc
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
#include <Eigen/Geometry> // Include this for additional Eigen functionalities
#include "json.hpp"
using json = nlohmann::json;

#define let auto const

// Function to calculate and return RRE (degrees) and RTE (meters).
auto compute_rre_rte(const Eigen::Matrix4d &current_transformation, const Eigen::Matrix4d &ground_truth_transformation) -> std::tuple<double, double>
{
  // Extract rotation matrices
  Eigen::Matrix3d R_current = current_transformation.block<3, 3>(0, 0);
  Eigen::Matrix3d R_ground_truth = ground_truth_transformation.block<3, 3>(0, 0);

  // Extract translation vectors
  Eigen::Vector3d t_current = current_transformation.block<3, 1>(0, 3);
  Eigen::Vector3d t_ground_truth = ground_truth_transformation.block<3, 1>(0, 3);

  // Calculate RTE as Euclidean distance between the two translation vectors
  double RTE = (t_current - t_ground_truth).norm();

  // Calculate RRE as the angle between the two rotation matrices
  Eigen::Quaterniond q_current(R_current), q_ground_truth(R_ground_truth);
  double RRE = q_current.angularDistance(q_ground_truth) * (180.0 / M_PI);

  return {RRE, RTE};
}

// Contains list of transform matrices organized frame 0 to end.
// Loads poses from a file.
auto load_poses(const std::string &path) -> std::vector<Eigen::Matrix4d>
{
  std::ifstream pose_file;
  pose_file.open(path);
  std::string line;
  std::vector<Eigen::Matrix4d> poses;
  std::string space = " ";
  while (std::getline(pose_file, line))
  {
    std::vector<double> elems;
    for (int row = 0; row < 4; row++)
    {
      for (int col = 0; col < 4; col++)
      {
        let elem = std::stod(line.substr(0, line.find(space)));
        line.erase(0, line.find(space) + 1);
        elems.push_back(elem);
      }
      if (row < 3)
      {
        std::getline(pose_file, line);
      }
    }
    auto const pose = Eigen::Map<Eigen::Matrix<double, 4, 4>>(elems.data());
    poses.push_back(pose);
  }
  pose_file.close();
  return poses;
}

// Returns a new matrix representing just positional info, with optional gaussian noise.
auto get_noisy_guess(const Eigen::Matrix4d &pose, double stddev = 0.0) -> Eigen::Matrix4d
{
  auto loc = Eigen::Vector3<double>{pose.block<1, 3>(3, 0).reshaped()};
  if (stddev > 0.0)
  {
    std::default_random_engine generator;
    std::normal_distribution<double> dist(0.0, stddev);
    // Modify only x and y coords
    loc(0) += dist(generator);
    loc(1) += dist(generator);
  }
  Eigen::Matrix4d new_pose;
  new_pose << 1.0, 0.0, 0.0, loc(0),
      0.0, 1.0, 0.0, loc(1),
      0.0, 0.0, 1.0, loc(2),
      0.0, 0.0, 0.0, 1.0;
  return new_pose;
}

const double GPS_NOISE_METERS = 10.0;
const int ICP_ITERS = 50;
const double SUCCESS_RTE = 2.0;

auto main(int argc, char *argv[]) -> int
{
  // For reproducability
  srand(100);

  // Checking program arguments
  if (argc < 2)
  {
    printf("Computes baseline data.\n\n");
    printf("Usage :\n");
    printf("  icp <data_root>\n");
    return (-1);
  }
  let data_root = std::string(argv[1]);

  double easy_total_rte = 0.0;
  double hard_total_rte = 0.0;
  double easy_total_rre = 0.0;
  double hard_total_rre = 0.0;
  int easy_total = 0;
  int hard_total = 0;
  int easy_success = 0;
  int hard_success = 0;
  double easy_time = 0.0;
  double hard_time = 0.0;

  for (let &ds_name : {"Dataset_1", "Dataset_2"})
  {
    for (let &sub_ds_name : {"D1", "D2", "D3", "D4", "D5"})
    {
      let folder_prefix = data_root + std::string("/") + std::string(ds_name) + std::string("/") + std::string(sub_ds_name) + std::string("/");

      // Load poses and metadata from folder
      let i_pose = load_poses(folder_prefix + "I_W.txt")[0];
      let v_poses = load_poses(folder_prefix + "V_W.txt");
      std::cout << folder_prefix << std::endl;
      std::ifstream f(folder_prefix + std::string("splits/info.json"));
      let data = json::parse(f);
      std::vector<int> hard_idxs = data["hard"];
      std::vector<int> easy_idxs = data["hard"];

      for (auto i = 0; i < 300; i++)
      {
        std::cout << i << "/300" << std::endl;
        let car_pcd = folder_prefix + std::string("splits/") + std::to_string(i) + std::string("_v.pcd");
        let infra_pcd = folder_prefix + std::string("splits/") + std::to_string(i) + std::string("_i.pcd");

        let iterations = ICP_ITERS;

        // Load car point cloud, transformed with noisy location
        PointCloudT::Ptr cloud_c(new PointCloudT);
        if (pcl::io::loadPCDFile(car_pcd, *cloud_c) < 0)
        {
          return -1;
        }
        let v_initial_guess = get_noisy_guess(v_poses[0], GPS_NOISE_METERS);
        pcl::transformPointCloud(*cloud_c, *cloud_c, v_initial_guess.transpose().cast<float>());

        // Load infra point cloud, transformed with ground truth
        PointCloudT::Ptr cloud_i(new PointCloudT);
        if (pcl::io::loadPCDFile(infra_pcd, *cloud_i) < 0)
        {
          return -1;
        }
        pcl::transformPointCloud(*cloud_i, *cloud_i, i_pose.transpose().cast<float>());

        // The Iterative Closest Point algorithm
        pcl::console::TicToc time;
        time.tic();
        pcl::IterativeClosestPoint<PointT, PointT> icp;
        icp.setMaximumIterations(iterations);
        icp.setInputTarget(cloud_i);
        icp.setInputSource(cloud_c);
        PointCloudT::Ptr cloud_final(new PointCloudT);
        icp.align(*cloud_final);
        let elapsed = time.toc();

        if (icp.hasConverged())
        {
          let new_car_pose = Eigen::Matrix4d(icp.getFinalTransformation().cast<double>());

          let[rre, rte] = compute_rre_rte(new_car_pose, v_poses[i].transpose());
          if (std::find(easy_idxs.begin(), easy_idxs.end(), i) != easy_idxs.end())
          {
            easy_total_rte += rte;
            easy_total_rre += rre;
            if (rte <= SUCCESS_RTE)
            {
              easy_success += 1;
            }
            easy_time += elapsed;
            easy_total += 1;
          }
          else
          {
            hard_total_rte += rte;
            hard_total_rre += rre;
            if (rte <= SUCCESS_RTE)
            {
              hard_success += 1;
            }
            hard_time += elapsed;
            hard_total += 1;
          }
        }
        else
        {
          PCL_ERROR("\nICP has not converged.\n");
        }
      }
      std::cout << "Avg. RTE (Easy): " << (float)easy_total_rte / (float)easy_total << std::endl;
      std::cout << "Avg. RRE (Easy): " << (float)easy_total_rre / (float)easy_total << std::endl;
      std::cout << "Success Rate (Easy): " << (float)easy_success / (float)easy_total << std::endl;
      std::cout << "Avg. Time (Easy): " << (float)easy_time / (float)easy_total << std::endl;
      std::cout << "Avg. RTE (Hard): " << (float)hard_total_rte / (float)hard_total << std::endl;
      std::cout << "Avg. RRE (Hard): " << (float)hard_total_rre / (float)hard_total << std::endl;
      std::cout << "Success Rate (Hard): " << (float)hard_success / (float)hard_total << std::endl;
      std::cout << "Avg. Time (Hard): " << (float)hard_time / (float)hard_total << std::endl;
    }
  }

  return 0;
}