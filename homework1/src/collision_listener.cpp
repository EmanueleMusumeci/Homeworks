#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <cstdlib>

class CollisionListener
{
	public:
		CollisionListener(bool voice)
		{
			this->voice = voice;
			this->lastTime = 0.0;
			this->lastDist = 0.0;
			sub = n.subscribe<std_msgs::Float32>("/min_dist", 1, &CollisionListener::collisionCallback, this);
		}
	
	private:
		ros::NodeHandle n;
		ros::Subscriber sub;
		bool voice;
		double lastTime;
		double lastDist;
		
		void collisionCallback(const std_msgs::Float32::ConstPtr& msg)
		{
			float dist = msg->data;
			if(lastDist==0.0) lastDist = dist;
			if(ros::Time::now().toSec() - lastTime < 1.0)
			{
				if(dist<0.2 && dist<=lastDist)
				{
					ROS_INFO("Collision!!!");
					system("espeak -v it \"ahia!\" -p 80");
				} 
				
				return;
			}
			else if(ros::Time::now().toSec() - lastTime < 2.0)
			{
				if(dist<0.2)
				{
					ROS_INFO("Collision!!!");
					system("espeak -v it \"ahi!\" -p 80");
				} 
				
				return;
			}
			else lastTime = ros::Time::now().toSec();
			if(dist<0.2)
			{
				ROS_INFO("Collision!!!");
				system("espeak -v it \"ahi\" -p 80");
			}
			/*else if(dist<1.0)
			{
				ROS_INFO("Collision imminent!!!");
				system("espeak -v it \"rallenta\" -p 80");
			}*/
		}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "collision_listener");

	bool voice_mode=false;
	
	/*if(argc!=2 || (argv[1][0]!='1' && argv[1][0]!='0'))
	{
		ROS_INFO("Usage: specify boolean parameter 1 or 0 for collision void mode on/off");
		return 0;
	}
	else voice_mode = argv[1];*/
	
	CollisionListener collision_listener(voice_mode);
	
	ros::spin();

	return 0;
}
