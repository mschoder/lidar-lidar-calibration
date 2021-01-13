#include <iostream>
#include <thread>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>

using namespace std::chrono_literals;

int
main (int argc, char** argv)
{
  // Loading first scan of room.
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  //if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/phil/subt_data/room_scan1.pcd", *target_cloud) == -1)
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/phil/subt_data/main_pcd/1582327002.030764000.pcd", *target_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read target file \n");
    return (-1);
  }
  std::cout << "Loaded " << target_cloud->size () << " data points from target point cloud" << std::endl;

  // Loading second scan of room from new perspective.
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  //if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/phil/subt_data/room_scan2.pcd", *input_cloud) == -1)
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/phil/subt_data/front_pcd/1582327001.992175000.pcd", *input_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read input file \n");
    return (-1);
  }
  std::cout << "Loaded " << input_cloud->size () << " data points from input point cloud" << std::endl;

  // Filtering input scan to roughly 10% of original size to increase speed of registration.
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize (0.05, 0.05, 0.05);
  approximate_voxel_filter.setInputCloud (input_cloud);
  approximate_voxel_filter.filter (*filtered_cloud);
  std::cout << "Filtered cloud contains " << filtered_cloud->size ()
            << " data points from input cloud" << std::endl;

  // Initializing Normal Distributions Transform (NDT).
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

  // Setting scale dependent NDT parameters
  // Setting minimum transformation difference for termination condition.
  ndt.setTransformationEpsilon (0.01);
  // Setting maximum step size for More-Thuente line search.
  ndt.setStepSize (0.1);
  //Setting Resolution of NDT grid structure (VoxelGridCovariance).
  ndt.setResolution (1.0);

  // Setting max number of registration iterations.
  ndt.setMaximumIterations (35);

  // Setting point cloud to be aligned.
  ndt.setInputSource (filtered_cloud);
  // Setting point cloud to be aligned to.
  ndt.setInputTarget (target_cloud);

  // Set initial alignment estimate found using robot odometry.
  // Front initial guess
  Eigen::AngleAxisf init_rotation_x(0, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf init_rotation_y(0, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf init_rotation_z(-M_PI/4, Eigen::Vector3f::UnitZ());
  Eigen::Translation3f init_translation (0.038, 0, 0.405);  


  // Rear initial guess
  // Eigen::AngleAxisf init_rotation_x(0, Eigen::Vector3f::UnitX());
  // Eigen::AngleAxisf init_rotation_y(0, Eigen::Vector3f::UnitY());
  // Eigen::AngleAxisf init_rotation_z(-M_PI-M_PI/4, Eigen::Vector3f::UnitZ());
  // Eigen::Translation3f init_translation (-0.47, 0, 0.21);

  Eigen::Matrix4f init_guess = (init_translation * init_rotation_x * init_rotation_y * init_rotation_z).matrix ();

  // Calculating required rigid transform to align the input cloud to the target cloud.
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  ndt.align (*output_cloud, init_guess);

  std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
            << " score: " << ndt.getFitnessScore () << std::endl;

  // print current transform
  Eigen::Matrix4f current_guess_;
  current_guess_ = ndt.getFinalTransformation();

  Eigen::Matrix3f rotation_matrix = current_guess_.block(0,0,3,3);
  Eigen::Vector3f translation_vector = current_guess_.block(0,3,3,1);
  std::cout << "This transformation can be replicated using:" << std::endl;
  std::cout << "rosrun tf static_transform_publisher " << translation_vector.transpose()
            << " " << rotation_matrix.eulerAngles(2,1,0).transpose() << " /" << "husky4/velodyne_points"
            << " /" << "husky4/velodyne_front/velodyne_points" << " 10" << std::endl;

  // Transforming unfiltered, input cloud using found transform.
  pcl::transformPointCloud (*input_cloud, *output_cloud, ndt.getFinalTransformation ());

  // Saving transformed input cloud.
  pcl::io::savePCDFileASCII ("room_scan2_transformed.pcd", *output_cloud);

  // Initializing point cloud visualizer
  pcl::visualization::PCLVisualizer::Ptr
  viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer_final->setBackgroundColor (0, 0, 0);

  // Coloring and visualizing target cloud (red).
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  target_color (target_cloud, 255, 0, 0);
  viewer_final->addPointCloud<pcl::PointXYZ> (target_cloud, target_color, "target cloud");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "target cloud");

  // Coloring and visualizing transformed input cloud (green).
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  output_color (output_cloud, 0, 255, 0);
  viewer_final->addPointCloud<pcl::PointXYZ> (output_cloud, output_color, "output cloud");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "output cloud");

  // Starting visualizer
  viewer_final->addCoordinateSystem (1.0, "global");
  viewer_final->initCameraParameters ();

  // Wait until visualizer window is closed.
  while (!viewer_final->wasStopped ())
  {
    viewer_final->spinOnce (100);
    std::this_thread::sleep_for(100ms);
  }

  return (0);
}