#include "dataset.h"
#include <fstream>
#include <iostream>

Dataset::Dataset(std::string& path){
    this->path = path;
    load_i_pose();
    load_c_poses();
}

Frame Dataset::getFrame(int frame_num){
    Frame frame(frame_num, this);
    return frame;
}

void Dataset::load_i_pose(){
  std::ifstream pose_file;
  pose_file.open((this->path+"I_W.txt"));
  std::string line;
  std::string space = " ";
  while (std::getline(pose_file, line)) {
    std::vector<double> elems;
    for (int row = 0; row < 4; row++) {
      for (int col = 0; col < 4; col++) {
        double elem = std::stod(line.substr(0, line.find(space)));
        line.erase(0, line.find(space) + 1);
        elems.push_back(elem);
      }
      if (row < 3) {
        std::getline(pose_file, line);
      }
    }
    this->i_pose = Eigen::Map<Eigen::Matrix<double, 4, 4>>(elems.data());
    pose_file.close();
  }
}

void Dataset::load_c_poses(){
    std::cout << "in load_c_poses" << std::endl;
    std::ifstream pose_file;
    pose_file.open((this->path+"V_W.txt"));
    std::string line;
    std::string space = " ";
    while (std::getline(pose_file, line)) {
        std::vector<double> elems;
        for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            double elem = std::stod(line.substr(0, line.find(space)));
            line.erase(0, line.find(space) + 1);
            elems.push_back(elem);
        }
        if (row < 3) {
            std::getline(pose_file, line);
        }
        }
        auto const pose = Eigen::Map<Eigen::Matrix<double, 4, 4>>(elems.data());
        std::cout << "Car Pose Read:\n" << pose << std::endl;
        this->c_poses.push_back(pose.transpose());
        this->c_poses_corrected.push_back(pose.transpose());
    }
    pose_file.close();
}
