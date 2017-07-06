#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "ros/ros.h"
#include <math.h>
#include <iostream>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>

#define PI 3.14159265

using namespace cv;

void scanCallback(const sensor_msgs::LaserScan latest_scan);
void odomCallback(const nav_msgs::Odometry latest_odom);
float ranges_min(sensor_msgs::LaserScan msg, int length);
float aux_min(const float* ranges, int length);
int ranges_length(sensor_msgs::LaserScan msg);
void draw_ranges(const sensor_msgs::LaserScan latest_scan, float min_dist);

double theta;
sensor_msgs::LaserScan latest_scan;
float min_dist;
int width, height;
double scaleFactor;

int main(int argc, char** argv) {

	ros::init(argc, argv, "output_window");
	ros::NodeHandle n;
	
	namedWindow("output_window", WINDOW_NORMAL);
	
	min_dist = 0;
	
	width = 500;
	height = width;
	scaleFactor = width/25;
	
	resizeWindow("output_window", width, height);
	
	ros::Subscriber laser_scan_sub = n.subscribe<sensor_msgs::LaserScan>("/base_scan", 1000, scanCallback);
	ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("/odom", 1000, odomCallback);
				
	ros::spin();
	
	return 0;
}

void scanCallback(const sensor_msgs::LaserScan latest_scan) {
	
	int n = ranges_length(latest_scan);
	
	float min = ranges_min(latest_scan, n);
	
	ROS_INFO("LaserScan message received. Length: %d\nMINIMUM VALUE: %g", n, min);

	if(min!=min_dist)
	{
		min_dist = min;
	}
	
	draw_ranges(latest_scan, min_dist);

}

void odomCallback(const nav_msgs::Odometry latest_odom) {
      
    tf::Pose pose;
	tf::poseMsgToTF(latest_odom.pose.pose, pose);
	theta = tf::getYaw(pose.getRotation()) * 180 / PI;
	
	ROS_INFO("Relative rotation: %f", theta);
}

int ranges_length(sensor_msgs::LaserScan msg)
{
	return (msg.angle_max - msg.angle_min)/msg.angle_increment;
}

float ranges_min(sensor_msgs::LaserScan msg, int length)
{
	const float* arr = &msg.ranges[0];
	
	return aux_min(arr, length);
}

float aux_min(const float* ranges, int length)
{
	float min=ranges[0];
	for(int i=0; i<length; i++) if(ranges[i]<min) min=ranges[i];
}

void draw_ranges(const sensor_msgs::LaserScan latest_scan, float min_dist) 
{

	int n = ranges_length(latest_scan);
	int offset = width/2;
	
    Mat img = Mat::zeros(width, height, CV_8UC3);
    img.setTo(cv::Scalar(255,255,255));
    
    Point * point_array = new Point[n];
    
    double angle = theta + latest_scan.angle_min * 180 / PI;
    
	for(int i=0; i<n; i++, angle+=latest_scan.angle_increment * 180 / PI)
	{
		int pos_x = offset + (cos(angle * PI / 180.0)*latest_scan.ranges[i])*scaleFactor*-1;
		int pos_y = offset + (sin(angle * PI / 180.0)*latest_scan.ranges[i])*scaleFactor;
		
		point_array[i] = Point(pos_x, pos_y);
		
		if(latest_scan.ranges[i] == min_dist) circle(img, point_array[i], scaleFactor, Scalar(0,0,255), 2);
	}
	
	for(int i=0; i<n-1; i++)
	{
		if(clipLine(Size(width, height), point_array[i], point_array[i+1])) {
			line(img, point_array[i], point_array[i+1], Scalar(0), 1);
		}
	}
	
    imshow( "output_window", img );	
    waitKey(30);
}
