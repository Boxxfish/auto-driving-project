#include <pcl/visualization/pcl_visualizer.h>
#include <string>
#include "visualizer.h"
 
void create_visualizer(const std::string& name, PointCloudT::Ptr cloud_i, PointCloudT::Ptr cloud_c_original, PointCloudT::Ptr cloud_c_transformed){
  // Visualization
  pcl::visualization::PCLVisualizer viewer (name);
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
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_c_color_h (cloud_c_transformed, 180, 20, 20);
  viewer.addPointCloud (cloud_c_transformed, cloud_c_color_h, "cloud_c_v2", v2);

  // Adding text descriptions in each viewport
  viewer.addText ("White: Infrasturcture Point Cloud\nGreen: Car Point Cloud Unaligned", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
  viewer.addText ("White: Infrasturcture Point Cloud\nRed: Car Point Cloud Aligned", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

  // Set background color
  viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
  viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

  // Set camera position and orientation
  // viewer.setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
  viewer.addCoordinateSystem (20.0);
  viewer.initCameraParameters ();
  viewer.setCameraPosition (100, 100, 100, 0, 0, 0, 0);

  int size = 3;
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size,"cloud_i_v1");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size,"cloud_i_v2");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size,"cloud_c_v1");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size,"cloud_c_v2");

  viewer.setSize (2500, 1600);  // Visualiser window size
  viewer.spin ();

  return;
}
 
