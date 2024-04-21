#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <Eigen/Geometry> // Include this for additional Eigen functionalities

struct Location {
    double x, y, z;
};

struct Rotation {
    double pitch, yaw, roll;
};


int parseLine(const std::string& line, Location& loc, Rotation& rot) {
  // std::cout << "in parse line" << std::endl;
  // std::cout << line << std::endl;

  std::istringstream iss(line);
  std::string token;

  int frame_num;
  
  // Get frame num
  iss >> token;
  frame_num = stoi(token);
  // std::cout << frame_num << std::endl;

  //ex token = "Transform(Location(x=-112.302399,"
  iss >> token;
  // std::cout << token << std::endl;
  std::string delimiter_start = "=";
  std::string delimiter_end = ",";
  token = token.substr(token.find(delimiter_start)+1, token.find(delimiter_end)-token.find(delimiter_start));
  loc.x = stod(token);
  // std::cout << loc.x << std::endl;

  //ex token = "y=2.822677,"
  iss >> token;
  // std::cout << token << std::endl;
  delimiter_start = "=";
  delimiter_end = ",";
  token = token.substr(token.find(delimiter_start)+1, token.find(delimiter_end)-token.find(delimiter_start));
  loc.y = stod(token);
  // std::cout << loc.y << std::endl;

  //ex token = "z=3.647835),"
  iss >> token;
  // std::cout << token << std::endl;
  delimiter_start = "=";
  delimiter_end = ")";
  token = token.substr(token.find(delimiter_start)+1, token.find(delimiter_end)-token.find(delimiter_start));
  loc.z = stod(token);
  // std::cout << loc.z << std::endl;

  //ex token = "Rotation(pitch=0.000000,"
  iss >> token;
  // std::cout << token << std::endl;
  delimiter_start = "=";
  delimiter_end = ",";
  token = token.substr(token.find(delimiter_start)+1, token.find(delimiter_end)-token.find(delimiter_start));
  rot.pitch = stod(token);
  // std::cout << rot.pitch << std::endl;


  //ex token = "yaw=-0.139465,"
  iss >> token;
  // std::cout << token << std::endl;
  delimiter_start = "=";
  delimiter_end = ",";
  token = token.substr(token.find(delimiter_start)+1, token.find(delimiter_end)-token.find(delimiter_start));
  rot.yaw = stod(token);
  // std::cout << rot.yaw << std::endl;

  //ex token = "roll=0.000000))"
  iss >> token;
  // std::cout << token << std::endl;
  delimiter_start = "=";
  delimiter_end = ")";
  token = token.substr(token.find(delimiter_start)+1, token.find(delimiter_end)-token.find(delimiter_start));
  rot.roll = stod(token);
  // std::cout << rot.roll << std::endl;

  return frame_num;
}

void generate_matrix(Eigen::Matrix4d& translation_matrix, const std::string& path, int data_num){
  std::ifstream file(path);
  std::string line;
  std::cout << "in generate matrix" << std::endl;

  while (std::getline(file, line)) {
    // std::cout << "reading a new line" << std::endl;
    Location loc;
    Rotation rot;
    int frame_num = parseLine(line, loc, rot);


    if (frame_num-1 == data_num){
      // Defining a rotation matrix and translation vector
      // Calculate translation matrix
      translation_matrix.block<3, 1>(0, 3) = Eigen::Vector3d(loc.x, loc.y, loc.z);
      std::cout << "Translation Matrix: \n" << translation_matrix << std::endl;

      // This is another way to get the rotation matrix, it is giving me the same result too

      // Calculate rotation matrix
      Eigen::AngleAxisd rollAngle(rot.roll, Eigen::Vector3d::UnitX());
      std::cout << "Roll Matrix: \n" << rollAngle.matrix() <<std::endl;
      Eigen::AngleAxisd yawAngle(rot.yaw, Eigen::Vector3d::UnitY());
      std::cout << "Yaw Matrix: \n" <<  yawAngle.matrix() <<std::endl;

      Eigen::AngleAxisd pitchAngle(rot.pitch, Eigen::Vector3d::UnitZ());
      std::cout << "Pitch Matrix: \n" << pitchAngle.matrix() <<std::endl;

      Eigen::Quaterniond quaternion = rollAngle * yawAngle * pitchAngle;
      std::cout << "Quaternion: \n" << quaternion << std::endl;
      Eigen::Matrix3d rotationMatrix = quaternion.toRotationMatrix();

      std::cout << "Rotation Matrix: \n" << rotationMatrix << std::endl;

      // combining the two matricies
      translation_matrix (0, 0 ) = rotationMatrix (0, 0);
      translation_matrix (0, 1 ) = rotationMatrix (0, 1);
      translation_matrix (0, 2 ) = rotationMatrix (0, 2);
      translation_matrix (1, 0 ) = rotationMatrix (1, 0);
      translation_matrix (1, 1 ) = rotationMatrix (1, 1);
      translation_matrix (1, 2 ) = rotationMatrix (1, 2);
      translation_matrix (2, 0 ) = rotationMatrix (2, 0);
      translation_matrix (2, 1 ) = rotationMatrix (2, 1);
      translation_matrix (2, 2 ) = rotationMatrix (2, 2);
      std::cout << "Final Transform Matrix: \n" << translation_matrix << std::endl;
      return;
    }

  }

  return;
}