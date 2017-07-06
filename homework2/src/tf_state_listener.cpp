#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"

class LaserListener
{
  private:
    ros::NodeHandle n;
    ros::Subscriber laser_subscriber;
    ros::Subscriber odom_subscriber;
    nav_msgs::Odometry * latest_odom_message;
    tf::StampedTransform latest_transform;
    tf::TransformListener transform_listener;

	public:
		LaserListener()
		{
      ROS_INFO("\n\n\nI'M HEEEERE!!!!\n\n\n");
			laser_subscriber = n.subscribe<sensor_msgs::LaserScan>("/base_scan", 1000, &LaserListener::scanCallback, this);
			odom_subscriber = n.subscribe<nav_msgs::Odometry>("/odom", 1000, &LaserListener::odomCallback, this);
      transform_listener.waitForTransform("/base_scan", "/base_link", ros::Time(0), ros::Duration(1.0));
    }

    void odomCallback(nav_msgs::Odometry msg)
    {
        latest_odom_message = &msg;
    }

		void scanCallback(sensor_msgs::LaserScan msg)
		{
      try {
          transform_listener.lookupTransform("/base_scan", "/base_link", ros::Time(0), latest_transform);
      }
      catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        return;
      }

      if(latest_odom_message==NULL) return;

      double timestamp_Sec = latest_transform.stamp_.sec;
      double timestamp_USec = ((uint64_t) latest_transform.stamp_.nsec - (uint64_t) latest_transform.stamp_.sec * 1e9)*1e-3;
      double xPosition = latest_transform.getOrigin().x() + latest_odom_message->pose.pose.position.x;
      double yPosition = latest_transform.getOrigin().y() + latest_odom_message->pose.pose.position.y;
      double angle = latest_transform.getRotation().getAngle() +
                    tf::Quaternion(latest_odom_message->pose.pose.orientation.x,
                    latest_odom_message->pose.pose.orientation.y,
                    latest_odom_message->pose.pose.orientation.z,
                    latest_odom_message->pose.pose.orientation.w).getAngle();


                    ROS_INFO("[Timestamp: %g,%g , Position {X: %g, Y: %g, Angle: %g}] LaserScan message received", timestamp_Sec, timestamp_USec, xPosition, yPosition, angle);
		}
};

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_state_listener");

  ROS_INFO("\n\n\nHEY THERE!!!!\n\n\n");

  LaserListener laser_listener;

	ros::spin();

  return 0;
};
