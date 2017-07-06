#include <ros/ros.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <sensor_msgs/PointCloud2.h>

void pointCloudCallback(const sensor_msgs::PointCloud2 pc2);

int main (int argc, char** argv) {

  ros::init(argc, argv, "homework6");

  ros::NodeHandle nh;

  ros::Subscriber cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1000, pointCloudCallback);

  ros::spin();

  return (0);
}

void pointCloudCallback(const sensor_msgs::PointCloud2 pc2) {

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg(pc2, *pcl);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_unfiltered (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer_unfiltered->setBackgroundColor (0, 0, 0);
  viewer_unfiltered->addPointCloud<pcl::PointXYZ> (pcl, "Unfiltered" );
  viewer_unfiltered->initCameraParameters ();
  viewer_unfiltered->addCoordinateSystem (1.0);
  viewer_unfiltered->spin();


  pcl::PointCloud<pcl::PointXYZ>::Ptr cut_cloud (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PassThrough<pcl::PointXYZ> pass_through;
  pass_through.setInputCloud (pcl);
  pass_through.setFilterLimits (0.0, 2.0);
  pass_through.setFilterFieldName ("z");
  pass_through.filter( *cut_cloud );

  pcl::PointCloud<pcl::PointXYZ>::Ptr voxelized_cloud (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setInputCloud (cut_cloud);
  voxel_grid.setLeafSize (0.01, 0.01, 0.01);
  voxel_grid.filter ( *voxelized_cloud ) ;

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_filtered (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer_filtered->setBackgroundColor (0, 0, 0);
  viewer_filtered->addPointCloud<pcl::PointXYZ> (voxelized_cloud, "Filtered" );
  viewer_filtered->initCameraParameters ();
  viewer_filtered->addCoordinateSystem (1.0);
  viewer_filtered->spin();

}
