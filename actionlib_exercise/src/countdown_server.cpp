#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib_exercise/CountdownAction.h>

class CountdownAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<actionlib_exercise::CountdownAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  actionlib_exercise::CountdownFeedback feedback_;
  actionlib_exercise::CountdownResult result_;

public:

  CountdownAction(std::string name) :
    as_(nh_, name, boost::bind(&CountdownAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~CountdownAction(void)
  {
  }

  void executeCB(const actionlib_exercise::CountdownGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // push_back the seeds for the fibonacci sequence
    feedback_.time_elapsed = goal->target_time;

    // publish info to the console for the user
    ROS_INFO("%s: Executing, creating countdown starting from %d", action_name_.c_str(), goal->target_time);

    // start executing the action
    for(int i=goal->target_time; i>=1; i--)
    {

      ROS_INFO("%d",feedback_.time_elapsed);

      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
      feedback_.time_elapsed-=1;
      // publish the feedback
      as_.publishFeedback(feedback_);

      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep();
    }

    if(success)
    {
      result_.time_elapsed = feedback_.time_elapsed;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "countdown");

  CountdownAction countdown("countdown");
  ros::spin();

  return 0;
}
