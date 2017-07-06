#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/LaserScan.h"

class LaserScan
{
	public:
		LaserScan();
		void scanCallback(sensor_msgs::LaserScan msg);
		sensor_msgs::LaserScan getLatestScan();

	private:
	
		ros::Subscriber sub;
		ros::Publisher pub;
		float prevMinDist;
		sensor_msgs::LaserScan latest_scan;
		
		int ranges_length(sensor_msgs::LaserScan msg);
		float ranges_min(sensor_msgs::LaserScan msg, int length);
		float aux_min(const float* ranges, int length);
		
};
