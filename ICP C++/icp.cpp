#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
#include <Eigen/Geometry> // Include this for additional Eigen functionalities

bool next_iteration = false;

void print4x4Matrix (const Eigen::Matrix4d & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event, void*)
{
  if (event.getKeySym () == "space" && event.keyDown ())
    next_iteration = true;
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

// Loads poses from a file.
std::vector<Eigen::Matrix4d> load_poses(std::string path) {
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
      std::getline(pose_file, line);
    }
    auto const pose = Eigen::Map<Eigen::Matrix<double, 4, 4>>(elems.data());
    poses.push_back(pose);
  }
  pose_file.close();
  return poses;
}

int main (int argc, char* argv[])
{
  // The point clouds we will be using
  PointCloudT::Ptr cloud_i (new PointCloudT);  // Original point cloud
  PointCloudT::Ptr cloud_c_original (new PointCloudT);  // Transformed point cloud
  PointCloudT::Ptr cloud_c (new PointCloudT);  // ICP output point cloud

  Eigen::Matrix4d cumulative_transformation = Eigen::Matrix4d::Identity(); // Initialize cumulative transformation

  // Checking program arguments
  if (argc < 3)
  {
    printf ("Usage :\n");
    printf ("\t\t%s split_data_number number_of_ICP_iterations\n", argv[0]);
    PCL_ERROR ("Provide one pcd file.\n");
    return (-1);
  }

  std::string car_pcd = std::string("../../splits/") + argv[1] + std::string("_v.pcd");
  std::string infra_pcd = std::string("../../splits/") + argv[1] + std::string("_i.pcd");

  int iterations = 1;  // Default number of ICP iterations
  if (argc > 3)
  {
    // If the user passed the number of iteration as an argument
    iterations = atoi (argv[3]);
    if (iterations < 1)
    {
      PCL_ERROR ("Number of initial iterations must be >= 1\n");
      return (-1);
    }
  }

  pcl::console::TicToc time;
  time.tic ();
  if (pcl::io::loadPCDFile (car_pcd, *cloud_c) < 0)
  {
    PCL_ERROR ("Error loading cloud %s.\n", argv[1]);
    return (-1);
  }
  std::cout << "\nLoaded file " << car_pcd << " (" << cloud_c->size () << " points) in " << time.toc () << " ms\n" << std::endl;

  time.tic ();
  if (pcl::io::loadPCDFile (infra_pcd, *cloud_i) < 0)
  {
    PCL_ERROR ("Error loading cloud %s.\n", argv[1]);
    return (-1);
  }
  std::cout << "\nLoaded file " << infra_pcd << " (" << cloud_i->size () << " points) in " << time.toc () << " ms\n" << std::endl;
  
  
  auto const poses = load_poses(std::string("poses.txt"));
  std::cout << "Loaded file " << argv[2] << " (" << poses.size() << " transforms)" << std::endl;

  // The Iterative Closest Point algorithm
  time.tic ();
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setMaximumIterations (iterations);
  icp.setInputSource (cloud_c); //cloud_c --> cloud_c
  icp.setInputTarget (cloud_i); //cloud_i --> i
  icp.align (*cloud_c);
  icp.setMaximumIterations (1);  // We set this variable to 1 for the next time we will call .align () function
  std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc () << " ms" << std::endl;

  if (icp.hasConverged ())
  {
    std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
    std::cout << "\nICP transformation " << iterations << " : cloud_c -> cloud_i" << std::endl;
  }
  else
  {
    PCL_ERROR ("\nICP has not converged.\n");
    return (-1);
  }

  *cloud_c_original = *cloud_c;

  // Visualization
  pcl::visualization::PCLVisualizer viewer ("ICP demo");
  // Create two vertically separated viewports
  int v1 (0);
  int v2 (1);
  viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
  viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);

  // The color we will be using
  float bckgr_gray_level = 0.0;  // Black
  float txt_gray_lvl = 1.0 - bckgr_gray_level;

  // Original point cloud is white
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_i_color_h (cloud_i, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl,
                                                                             (int) 255 * txt_gray_lvl);
  viewer.addPointCloud (cloud_i, cloud_i_color_h, "cloud_i_v1", v1);
  viewer.addPointCloud (cloud_i, cloud_i_color_h, "cloud_i_v2", v2);

  // Transformed point cloud is green
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_c_original_color_h (cloud_c_original, 20, 180, 20);
  viewer.addPointCloud (cloud_c_original, cloud_c_original_color_h, "cloud_c_v1", v1);

  // ICP aligned point cloud is red
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_c_color_h (cloud_c, 180, 20, 20);
  viewer.addPointCloud (cloud_c, cloud_c_color_h, "cloud_c_v2", v2);

  // Adding text descriptions in each viewport
  viewer.addText ("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
  viewer.addText ("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

  std::stringstream ss;
  ss << iterations;
  std::string iterations_cnt = "ICP iterations = " + ss.str ();
  viewer.addText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);

  // Set background color
  viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
  viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

  // Set camera position and orientation
  // viewer.setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
  viewer.addCoordinateSystem (1.0);
  viewer.initCameraParameters ();
  viewer.setCameraPosition (100, 100, 100, 0, 0, 0, 0);

  viewer.setSize (2500, 1600);  // Visualiser window size

  // Register keyboard callback :
  viewer.registerKeyboardCallback (&keyboardEventOccurred, (void*) NULL);

  // Display the visualiser
  while (!viewer.wasStopped ())
  {
    viewer.spin ();

    // The user pressed "space" :
    if (next_iteration)
    {
      // The Iterative Closest Point algorithm
      time.tic ();
      icp.align (*cloud_c);
      std::cout << "Applied 1 ICP iteration in " << time.toc () << " ms" << std::endl;

      if (icp.hasConverged ())
      {
        printf ("\033[11A");  // Go up 11 lines in terminal output.
        printf ("\nICP has converged, score is %+.0e\n", icp.getFitnessScore ());
        std::cout << "\nICP transformation " << ++iterations << " : cloud_c -> cloud_i" << std::endl;
        // transformation_matrix *= icp.getFinalTransformation ().cast<double>();  // WARNING /!\ This is not accurate! For "educational" purpose only!
        // print4x4Matrix (transformation_matrix);  // Print the transformation between original pose and current pose

        cumulative_transformation *= icp.getFinalTransformation().cast<double>(); // Update cumulative transformation
        print4x4Matrix(cumulative_transformation); // Print the current cumulative transformation matrix
        // printRREandRTE(cumulative_transformation, transformation_matrix); // Assuming 'transformation_matrix' is your ground truth

        ss.str ("");
        ss << iterations;
        std::string iterations_cnt = "ICP iterations = " + ss.str ();
        viewer.updateText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
        viewer.updatePointCloud (cloud_c, cloud_c_color_h, "cloud_c_v2");
      }
      else
      {
        PCL_ERROR ("\nICP has not converged.\n");
        return (-1);
      }
    }
    next_iteration = false;
  }
  return (0);
}