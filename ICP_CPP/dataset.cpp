#include "dataset.h"
#include <fstream>
#include <iostream>
#include "json.hpp"

using json = nlohmann::json;

Dataset::Dataset(const std::string &path)
{
  // Load poses
  this->i_pose = read_poses(path + "I_W.txt")[0];
  auto const c_poses = read_poses(path + "V_W.txt");

  // Load point clouds and store as frames
  for (int i = 0; i < 300; i++)
  {
    auto const splits_path_prefix = path + "splits/" + std::to_string(i);
    auto const cloud_i = load_pc(splits_path_prefix + "_i.pcd");
    auto const cloud_c = load_pc(splits_path_prefix + "_v.pcd");
    this->frames.push_back({
        .cloud_i = cloud_i,
        .cloud_c = cloud_c,
        .pose_c = c_poses[i],
    });
  }

  // Load easy and hard indices
  std::ifstream f(path + std::string("splits/info.json"));
  auto data = json::parse(f);
  std::vector<int> hard_idxs = data["hard"];
  std::vector<int> easy_idxs = data["easy"];
}

std::vector<Eigen::Matrix4d> read_poses(const std::string &path)
{
  std::ifstream pose_file;
  pose_file.open(path);

  std::string line;
  std::string space = " ";
  std::vector<Eigen::Matrix4d> poses;
  while (std::getline(pose_file, line))
  {
    std::vector<double> elems;
    for (int row = 0; row < 4; row++)
    {
      for (int col = 0; col < 4; col++)
      {
        double elem = std::stod(line.substr(0, line.find(space)));
        line.erase(0, line.find(space) + 1);
        elems.push_back(elem);
      }
      if (row < 3)
      {
        std::getline(pose_file, line);
      }
    }
    auto const pose = Eigen::Map<Eigen::Matrix4d>(elems.data());
    poses.push_back(pose.transpose());
  }
  pose_file.close();

  return poses;
}

PointCloudT::Ptr load_pc(const std::string &path)
{
  PointCloudT::Ptr cloud_temp(new PointCloudT);

  if (pcl::io::loadPCDFile(path, *(cloud_temp)) < 0)
  {
    PCL_ERROR("Error loading cloud.\n");
    exit(-1);
  }
  return cloud_temp;
}