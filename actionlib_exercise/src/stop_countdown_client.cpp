#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_exercise/CountdownAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "stop_countdown");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<actionlib_exercise::CountdownAction> ac("countdown", true);

  ROS_INFO("Checking if action server already started.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, stopping countdown.");
  // send a goal to the action
  ac.cancelAllGoals();

  actionlib::SimpleClientGoalState state = ac.getState();
  ROS_INFO("Action finished: %s",state.toString().c_str());

  //exit
  return 0;
}
