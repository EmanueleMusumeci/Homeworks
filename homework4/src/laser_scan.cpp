#include "laser_scan.h"

int ranges_length(const sensor_msgs::LaserScan::ConstPtr& msg);
float ranges_min(const sensor_msgs::LaserScan::ConstPtr& msg, int length);
float aux_min(const float* ranges, int start, int end);

LaserScan::LaserScan()
{
	ros::init(argc, argv, "laser_scan");
	ros::NodeHandle n;
	
	sub = n.subscribe<sensor_msgs::LaserScan>("/base_scan", 1000, &LaserScan::scanCallback, this);
	
	prevMinDist = 0;
	
	pub = n.advertise<std_msgs::Float32>("/min_dist", 1);

}

void LaserScan::scanCallback(sensor_msgs::LaserScan msg)
{
	int n = ranges_length(msg);
	
	float min = ranges_min(msg, n);
	
	ROS_INFO("LaserScan message received. Length: %d\nMINIMUM VALUE: %g", n, min);

	if(min!=prevMinDist)
	{
		std_msgs::Float32 responseMsg;
	
		responseMsg.data = min;
	
		pub.publish(responseMsg);
	
		ros::spinOnce();
		
		prevMinDist = min;
	}
}

sensor_msgs::LaserScan LaserScan::getLatestScan()
{
	return this->latest_scan;
}

int LaserScan::ranges_length(sensor_msgs::LaserScan msg)
{
	return (msg.angle_max - msg.angle_min)/msg.angle_increment;
}

float LaserScan::ranges_min(sensor_msgs::LaserScan msg, int length)
{
	const float* arr = &msg.ranges[0];
	
	return aux_min(arr, length);
}

float LaserScan::aux_min(const float* ranges, int length)
{
	float min=ranges[0];
	for(int i=0; i<length; i++) if(ranges[i]<min) min=ranges[i];
}
