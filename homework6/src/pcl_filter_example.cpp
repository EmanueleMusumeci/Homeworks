#include <ros/ros.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

#include <vector>
#include <ctime>

int main (int argc, char** argv) {

  srand ((unsigned int) time (NULL));

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  cloud->width=2000;
  cloud->height=1;
  //cloud->isdense=false;
  cloud->points.resize(cloud->width*cloud->height);
  for(size_t i=0; i<cloud->points.size(); i++){

    cloud->points[i].x=10*rand()/(RAND_MAX+1.0);
    cloud->points[i].y=10*rand()/(RAND_MAX+1.0);
    cloud->points[i].z=10*rand()/(RAND_MAX+1.0);
  }

  pcl::io::savePCDFileASCII("test_pcd.pcd",*cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("test_pcd.pcd", *cloud2) == -1) //* load the file
  {
   PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
   return (-1);
  }
  std::cout << "Loaded "
           << cloud2->width * cloud2->height
           << " data points from test_pcd.pcd with the following fields: "
           << std::endl;

  for (size_t i = 0; i < cloud2->points.size (); ++i)
   std::cout << "    " << cloud2->points[i].x
             << " "    << cloud2->points[i].y
             << " "    << cloud2->points[i].z << std::endl;

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> ( cloud2, "Input cloud" );
  viewer->initCameraParameters ();
  viewer->addCoordinateSystem (1.0);
  while (!viewer->wasStopped ()) viewer->spinOnce ( 1 );

  pcl::PointCloud<pcl::PointXYZ>::Ptr cut_cloud (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PassThrough<pcl::PointXYZ> pass_through;
  pass_through.setInputCloud (cloud2);
  pass_through.setFilterLimits (0.0, 0.5);
  pass_through.setFilterFieldName ("z");
  pass_through.filter( *cut_cloud );

  std::cout << "Cloud after applying a pass_through filter along axis z";

  for (size_t i = 0; i < cut_cloud->points.size (); ++i)
  std::cout << "    " << cut_cloud->points[i].x
      << " "    << cut_cloud->points[i].y
      << " "    << cut_cloud->points[i].z << std::endl;

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2 (new pcl::visualization::PCLVisualizer ("PassThrough"));
  viewer2->setBackgroundColor (0, 0, 0);
  viewer2->addPointCloud<pcl::PointXYZ> ( cut_cloud, "PassThrough" );
  viewer2->initCameraParameters ();
  viewer2->addCoordinateSystem (1.0);
  while (!viewer2->wasStopped ()) viewer2->spinOnce ( 1 );

  pcl::PointCloud<pcl::PointXYZ>::Ptr voxelized_cloud (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setInputCloud (cut_cloud);
  voxel_grid.setLeafSize (0.01, 0.01, 0.01);
  voxel_grid.filter ( *voxelized_cloud ) ;

  std::cout << "Cloud after applying a voxelization filter along axis z";

  for (size_t i = 0; i < voxelized_cloud->points.size (); ++i)
  std::cout << "    " << voxelized_cloud->points[i].x
      << " "    << voxelized_cloud->points[i].y
      << " "    << voxelized_cloud->points[i].z << std::endl;

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer3 (new pcl::visualization::PCLVisualizer ("Voxelization"));
  viewer3->setBackgroundColor (0, 0, 0);
  viewer3->addPointCloud<pcl::PointXYZ> ( voxelized_cloud, "Voxelization" );
  viewer3->initCameraParameters ();
  viewer3->addCoordinateSystem (1.0);
  while (!viewer3->wasStopped ()) viewer3->spinOnce ( 1 );

  return (0);
}
