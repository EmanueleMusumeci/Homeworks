#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <homework5/Homework5Action.h>
#include <math.h>

class Homework5Action
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<homework5::Homework5Action> as_;
  // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  homework5::Homework5Feedback feedback_;
  homework5::Homework5Result result_;

  ros::Subscriber odom_subscriber;
  ros::Publisher vel_publisher;

  nav_msgs::Odometry latest_odom;
  nav_msgs::Odometry initial_odom;

public:

  Homework5Action(std::string name) :
    as_(nh_, name, boost::bind(&Homework5Action::executeCB, this, _1), false),
    action_name_(name)
  {
    odom_subscriber = nh_.subscribe<nav_msgs::Odometry>("/odom", 1000, &Homework5Action::odomCallback, this);
    initial_odom = *(ros::topic::waitForMessage<nav_msgs::Odometry>("odom", nh_));
    latest_odom = initial_odom;
    vel_publisher = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    as_.start();
  }

  ~Homework5Action(void)
  {
  }

  void odomCallback(const nav_msgs::Odometry msg) {
    latest_odom = msg;
//    ROS_INFO("Received new odom message with position (%f,%f,%f)",latest_odom.pose.pose.position.x, latest_odom.pose.pose.position.y, latest_odom.pose.pose.position.z);
//    ROS_INFO("Initial position is (%f,%f,%f)",initial_odom.pose.pose.position.x,initial_odom.pose.pose.position.y,initial_odom.pose.pose.position.z);
  }

  void executeCB(const homework5::Homework5GoalConstPtr &goal)
  {
    // helper variables
    bool success = true;
    ros::Rate r(30);
    // push_back the seeds for the fibonacci sequence
    // publish info to the console for the user
    ROS_INFO("%s: Executing, starting to move with desired_speed %f trying to reach distance %f", action_name_.c_str(), goal->desired_speed, goal->distance);

    float distance_moved = abs(latest_odom.pose.pose.position.x - initial_odom.pose.pose.position.x);

    // start executing the action
    while(distance_moved<goal->distance)
    {
      ROS_INFO("Distance moved: %f", distance_moved);

      geometry_msgs::Twist new_msg;

      new_msg.linear.x = goal->desired_speed;

      vel_publisher.publish(new_msg);

      ROS_INFO("Issued desired_speed %f command", goal->desired_speed);
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }

      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
    r.sleep();

    distance_moved = abs(latest_odom.pose.pose.position.x - initial_odom.pose.pose.position.x);

    }

    geometry_msgs::Twist new_msg;
    new_msg.linear.x = 0.0;
    vel_publisher.publish(new_msg);

    if(success)
    {
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded

      result_.odom_pose = latest_odom;

      as_.setSucceeded(result_);
    }

    exit(0);
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "homework5_server");

  Homework5Action homework5("homework5_server");
  ros::spin();

  return 0;
}
