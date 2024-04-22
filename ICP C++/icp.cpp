#include <iostream>
#include <string>
#include <random>
#include <typeinfo>
#include <fstream>
#include <sstream>
#include <cmath> // for radians conversion

#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
#include <Eigen/Geometry> // Include this for additional Eigen functionalities

#include "visualizer.h"
#include "dataset.h"
#include "frame.h"

bool next_iteration = false;

struct Location {
    double x, y, z;
};

struct Rotation {
    double pitch, yaw, roll;
};

void print4x4Matrix (const Eigen::Matrix4d & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

// Function to calculate and print RRE and RTE
void printRREandRTE(const Eigen::Matrix4d &current_transformation, const Eigen::Matrix4d &ground_truth_transformation) {
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
    double RRE = q_current.angularDistance(q_ground_truth);

    // Print RRE and RTE
    std::cout << "RTE: " << RTE << " meters" << std::endl;
    std::cout << "RRE: " << RRE * (180.0 / M_PI) << " degrees" << std::endl; // Convert RRE from radians to degrees
}

// contains a list of transofrm matricies organized frame 0 to end
// Loads poses from a file.
std::vector<Eigen::Matrix4d> load_poses(const std::string& path) {
  std::ifstream pose_file;
  pose_file.open(path);
  std::string line;
  std::vector<Eigen::Matrix4d> poses;
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
    poses.push_back(pose.transpose());
  }
  pose_file.close();
  return poses;
}

// Returns the location from a pose, with optional gaussian noise.
Eigen::Matrix<double, 3, 1> get_loc(const Eigen::Matrix4d& pose, double stddev = 0.0) {
  auto loc = Eigen::Vector3<double> { pose.block<1, 3>(3, 0).reshaped() };
  if (stddev > 0.0) {
    std::default_random_engine generator;
    std::normal_distribution<double> dist(0.0, stddev);
    // Modify only x and z coords
    loc(0) += dist(generator);
    loc(2) += dist(generator);
  }
  return loc;
}

// Returns the location from a pose, with optional gaussian noise.
void add_noise_to_loc(Eigen::Matrix4d& pose, double stddev = 0.0) {
  if (stddev > 0.0) {
    std::default_random_engine generator;
    std::normal_distribution<double> dist(0.0, stddev);
    // Modify only x and z coords
    pose(0,3) += dist(generator);
    pose(1,3) += dist(generator);
  }
  return;
}


const double GPS_NOISE_METERS = 10.0;

int main (int argc, char* argv[])
{






  // // The point clouds we will be using
  // PointCloudT::Ptr cloud_i (new PointCloudT);  // infrastructure point cloud <-- we are trying to align to this
  // PointCloudT::Ptr cloud_c_original (new PointCloudT);  // original car point cloud
  // PointCloudT::Ptr cloud_c (new PointCloudT);  // aligned car point cloud

  // Eigen::Matrix4d cumulative_transformation = Eigen::Matrix4d::Identity(); // Initialize cumulative transformation

  // Checking program arguments
  if (argc < 3)
  {
    printf ("ICP Point Cloud Merger.\n\n");
    printf ("Usage :\n");
    printf ("  icp <split_num> <num_ICP_iterations>\n");
    return (-1);
  }

  // std::string car_pcd = std::string("../../splits/") + argv[1] + std::string("_v.pcd");
  // std::string infra_pcd = std::string("../../splits/") + argv[1] + std::string("_i.pcd");
  int data_num = atoi(argv[1]);

  std::string path = "../../data/Dataset_1/D1/";
  Dataset dataset1_d1(path);
  std::cout << "Dataset Object Created." << std::endl;
  std::cout << "Infrastructure Pose:\n" << dataset1_d1.i_pose <<std::endl;
  std::cout << "Car Pose:\n" << dataset1_d1.c_poses[data_num] << std::endl;

  std::cout << "Loading Frame" << std::endl;
  Frame frame = dataset1_d1.getFrame(data_num);
  std::cout << "Frame Loaded" << std::endl;
  create_visualizer(std::string("Demo Visualizer"), frame.cloud_i, frame.cloud_c, frame.cloud_c);

  return 0;

  // int iterations = 1;  // Default number of ICP iterations

  // // If the user passed the number of iteration as an argument
  // iterations = atoi (argv[2]);
  // std::cout << iterations;
  // if (iterations < 1)
  // {
  //   PCL_ERROR ("Number of initial iterations must be >= 1\n");
  //   return (-1);
  // }
  
  // pcl::console::TicToc time;
  // time.tic ();
  // if (pcl::io::loadPCDFile (car_pcd, *cloud_c) < 0)
  // {
  //   PCL_ERROR ("Error loading cloud %s.\n", argv[1]);
  //   return (-1);
  // }
  // std::cout << "\nLoaded file " << car_pcd << " (" << cloud_c->size () << " points) in " << time.toc () << " ms\n" << std::endl;

  // time.tic ();
  // if (pcl::io::loadPCDFile (infra_pcd, *cloud_i) < 0)
  // {
  //   PCL_ERROR ("Error loading cloud %s.\n", argv[1]);
  //   return (-1);
  // }
  // std::cout << "\nLoaded file " << infra_pcd << " (" << cloud_i->size () << " points) in " << time.toc () << " ms\n" << std::endl;

  // std::vector<Eigen::Matrix4d> poses_i = load_poses(std::string("../../splits/I_W.txt"));
  // std::cout << "Loaded file " << "poses_i.txt" << " (" << poses_i.size() << " transforms)\n" << std::endl;
  // std::cout << poses_i[0] << std::endl;

  // std::vector<Eigen::Matrix4d> poses_c = load_poses(std::string("../../splits/V_W.txt"));
  // std::cout << "Loaded file " << "poses_c.txt" << " (" << poses_c.size() << " transforms)\n" << std::endl;
  // std::cout << poses_c[data_num] << std::endl;
  // std::cout << "Adding noise" <<std::endl;
  // add_noise_to_loc(poses_c[data_num],3);
  // std::cout << poses_c[data_num] << std::endl;


  // pcl::transformPointCloud (*cloud_i, *cloud_i, poses_i[0]);
  // pcl::transformPointCloud (*cloud_c, *cloud_c, poses_c[data_num]);


  // pcl::PassThrough<pcl::PointXYZ> pass_c;
  // pass_c.setInputCloud(cloud_c);
  // pass_c.setFilterFieldName("z");
  // pass_c.setFilterLimits(-0.3,0.3);
  // pass_c.setNegative(true);
  // pass_c.filter(*cloud_c);


  // pcl::PassThrough<pcl::PointXYZ> pass_i;
  // pass_i.setInputCloud(cloud_i);
  // pass_i.setFilterFieldName("z");
  // pass_i.setFilterLimits(-0.3,0.3);
  // pass_i.setNegative(true);
  // pass_i.filter(*cloud_i);

  // *cloud_c_original = *cloud_c; //create two different car point clouds for comon

  // std:cout << "ICP Starting\n";

  // //The Iterative Closest Point algorithm
  // time.tic ();
  // pcl::IterativeClosestPoint<PointT, PointT> icp;
  // icp.setMaximumIterations (iterations);
  // icp.setMaxCorrespondenceDistance(3);
  // icp.setInputSource (cloud_c); //cloud_c --> cloud_c
  // icp.setInputTarget (cloud_i); //cloud_i --> i
  // icp.align (*cloud_c);

  // std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc () << " ms" << std::endl;

  // if (icp.hasConverged ())
  // {
  //   std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
  //   std::cout << "\nICP transformation " << iterations << " : cloud_c -> cloud_i" << std::endl;
  // }
  // else
  // {
  //   PCL_ERROR ("\nICP has not converged.\n");
  //   return (-1);
  // }




  // time.tic ();
  // pcl::IterativeClosestPoint<PointT, PointT> icp;
  // icp.setMaximumIterations (1);
  // icp.setMaxCorrespondenceDistance(3);
  // icp.setInputSource (cloud_c); //cloud_c --> cloud_c
  // icp.setInputTarget (cloud_i); //cloud_i --> i
  // int i;
  // for (i=0; i < iterations; i++){
  //   icp.align (*cloud_c);
  //   std::cout << "ICP Iteration: " << i << "  ICP score is " << icp.getFitnessScore () << std::endl;
  //   if (icp.getFitnessScore() < 0.5){
  //     break;
  //   }
  // }
  // std::cout << "Applied " << i << " ICP iteration(s) in " << time.toc () << " ms" << std::endl;
  // std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
  // std::cout << "\nICP transformation " << iterations << " : cloud_c -> cloud_i" << std::endl;


  // create_visualizer(std::string("Demo Visualizer"), cloud_i, cloud_c_original, cloud_c);

  // return (0);
}