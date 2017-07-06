#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/LaserScan.h"

int ranges_length(const sensor_msgs::LaserScan::ConstPtr& msg);
float ranges_min(const sensor_msgs::LaserScan::ConstPtr& msg, int length);
float aux_min(const float* ranges, int start, int end);

class LaserListener
{
	public:
		LaserListener()
		{
			sub = n.subscribe<sensor_msgs::LaserScan>("/base_scan", 1000, &LaserListener::scanCallback, this);
			
			prevMinDist = 0;
			
			pub = n.advertise<std_msgs::Float32>("/min_dist", 1);
		}

		void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
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


	private:
	
		ros::NodeHandle n;
		ros::Subscriber sub;
		ros::Publisher pub;
		float prevMinDist;
		
		int ranges_length(const sensor_msgs::LaserScan::ConstPtr& msg)
		{
			return (msg->angle_max - msg->angle_min)/msg->angle_increment;
		}

		float ranges_min(const sensor_msgs::LaserScan::ConstPtr& msg, int length)
		{
			const float* arr = &msg->ranges[0];
			
			return aux_min(arr, length);
		}

		float aux_min(const float* ranges, int length)
		{
			float min=ranges[0];
			for(int i=0; i<length; i++) if(ranges[i]<min) min=ranges[i];
		}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "laser_listener");
	
	LaserListener laser_listener;
		
	ros::spin();
	
	return 0;
}
