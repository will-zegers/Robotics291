#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <speedydug_servers/SpinAction.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>

#define ANG_VEL 0.3141592653

class SpinAction
{
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<speedydug_servers::SpinAction> as_;
  std::string action_name_;
  // create messages that are used to published feedback/result
  speedydug_servers::SpinFeedback feedback_;
  speedydug_servers::SpinResult result_;
  ros::Subscriber point_sub_;
  ros::Publisher twist_pub_;
  geometry_msgs::Twist twist_;
  bool ball_detected_;

public:

  SpinAction(std::string name) :
	  as_(nh_, name, boost::bind(&SpinAction::executeCB, this, _1), false),
	  action_name_(name)
  {
	// register the goal and feeback callbacks
	point_sub_ = nh_.subscribe("/track_point", 1, &SpinAction::trackPointCB, this);
	twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	ball_detected_=false;

    as_.start();
  }

  ~SpinAction(void)
  {
  }


  void executeCB(const speedydug_servers::SpinGoalConstPtr &goal)
  {
	  // helper variables
	  ball_detected_ = false;
      feedback_.success = false;
	  float z;
	  ros::Duration d(0.05);

	  if( goal->left ){
		  z = ANG_VEL;
	  }
	  else {
		  z = -1.0 * ANG_VEL;
      }
	  ROS_INFO(goal->left ? "true" : "false");
	  ROS_INFO("z = %f", z);

	  while(!ball_detected_)
	  {
		  // check that preempt has not been requested by the client
		  if (as_.isPreemptRequested() || !ros::ok())
		  {
			  twist_.angular.z = 0;
			  twist_.linear.x = 0;
			  twist_pub_.publish(twist_);
			  ROS_INFO("%s: Preempted", action_name_.c_str());
			  // set the action state to preempted
			  as_.setPreempted();
			  break;
		  }

		  twist_.angular.z = z;
		  twist_.linear.x = 0;
		  twist_pub_.publish(twist_);
		  as_.publishFeedback(feedback_);
		  d.sleep();
	  }

	  if( ball_detected_ )
	  {
		  twist_.angular.z = 0;
		  twist_.linear.x = 0;
		  twist_pub_.publish(twist_);
		  result_.success = true;
		  ROS_INFO("%s: Succeeded", action_name_.c_str());
		  // set the action state to succeeded
		  as_.setSucceeded(result_);
	  }
  }

  void trackPointCB(const geometry_msgs::Point::ConstPtr& msg)
  {
	  // make sure that the action hasn't been canceled
	  if (!as_.isActive())
		  return;
	  ball_detected_ = true;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "spin");

  SpinAction spin("spin");//ros::this_node::getName());
  ros::spin();

  return 0;
}
