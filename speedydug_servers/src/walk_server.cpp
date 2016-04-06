#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <speedydug_servers/WalkAction.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

#define LIN_VEL 0.1

class WalkAction
{
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<speedydug_servers::WalkAction> as_; 
  std::string action_name_;
  // create messages that are used to published feedback/result
  speedydug_servers::WalkFeedback feedback_;
  speedydug_servers::WalkResult result_;
  ros::Subscriber point_sub_;
  ros::Subscriber us_sub_;
  ros::Publisher twist_pub_;
  geometry_msgs::Twist twist_;
  bool ball_detected_;
  bool obstacle_detected_;

public:

  WalkAction(std::string name) :
	  as_(nh_, name, boost::bind(&WalkAction::executeCB, this, _1), false),
	  action_name_(name)
  {
	// register the goal and feeback callbacks
	point_sub_ = nh_.subscribe("/track_point", 1, &WalkAction::trackPointCB, this);
	us_sub_ = nh_.subscribe("/obstacle", 1, &WalkAction::ultraSonicsCB, this);
	twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	ball_detected_ = false;
  	obstacle_detected_ = false;

    as_.start();
  }

  ~WalkAction(void)
  {
  }

  void executeCB(const speedydug_servers::WalkGoalConstPtr &goal)
  {
	  // helper variables
	  ball_detected_ = false;
	  obstacle_detected_ = false;
      feedback_.success = false;
	  ros::Duration d(0.05);

	  while(!ball_detected_ && !obstacle_detected_)
	  {
		  // check that preempt has not been requested by the client
		  if (as_.isPreemptRequested() || !ros::ok())
		  {
			  ROS_INFO("%s: Preempted", action_name_.c_str());
			  // set the action state to preempted
			  as_.setPreempted();
			  break;
		  }

		  twist_.angular.z = 0.0;
		  twist_.linear.x = LIN_VEL;
		  twist_pub_.publish(twist_);
		  d.sleep();
	  }

	  twist_.angular.z = 0.0;
	  twist_.linear.x = 0.0;
	  twist_pub_.publish(twist_);

	  if( ball_detected_ )
	  {
		  result_.success = true;
		  ROS_INFO("%s: Ball Detected", action_name_.c_str());
		  // set the action state to succeeded
		  as_.setSucceeded(result_);
	  }

	  else if( obstacle_detected_ )
	  {
		  result_.success = false;
		  ROS_INFO("%s: Obsticle Detected", action_name_.c_str());
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

  void ultraSonicsCB(const std_msgs::Int32::ConstPtr& dist)
  {
	  // make sure that the action hasn't been canceled
	  if (!as_.isActive()) return;
	  obstacle_detected_ = false;
	  if ( dist->data > 0 ){
		  obstacle_detected_ = true;
	  }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "walk");

  WalkAction walk("walk");//ros::this_node::getName());
  ros::spin();

  return 0;
}
