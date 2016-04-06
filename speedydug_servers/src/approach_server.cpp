#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <speedydug_servers/ApproachAction.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>

#define LIN_VEL 0.1
#define ANG_VEL 0.0
#define LIN_VEL_BASE 0.011
#define ANG_VEL_BASE 0.1
#define CAM_WIDTH 320
#define CAM_HEIGHT 240

class ApproachAction
{
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<speedydug_servers::ApproachAction> as_; 
  std::string action_name_;
  // create messages that are used to published feedback/result
  speedydug_servers::ApproachFeedback feedback_;
  speedydug_servers::ApproachResult result_;
  ros::Subscriber point_sub_;
  ros::Publisher twist_pub_;
  geometry_msgs::Twist twist_;
  bool has_goal_;

public:

  ApproachAction(std::string name) :
    as_(nh_, name, false),
    action_name_(name)
  {
	// register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&ApproachAction::goalCB, this));
	as_.registerPreemptCallback(boost::bind(&ApproachAction::preemptCB, this));
	twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	point_sub_ = nh_.subscribe("/track_point", 1, &ApproachAction::trackPointCB, this);
    as_.start();
  }

  ~ApproachAction(void)
  {
  }

  void goalCB()
  {
    // accept the new goal
    has_goal_ = as_.acceptNewGoal()->approach;
  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }

  // Helper methods for proportional control 
  float getAngVel(int xCurr, int xGoal){
	float prop;
	prop = fabs(xGoal - xCurr)/(0.5*CAM_WIDTH);
	return prop * ANG_VEL + ANG_VEL_BASE;
  }

  float getLinVel(int yCurr, int yGoal){
	float prop;
    prop = fabs(yGoal - yCurr)/(0.5*CAM_HEIGHT);
	return prop * LIN_VEL + LIN_VEL_BASE;
  }

  void trackPointCB(const geometry_msgs::Point::ConstPtr& msg)
  {
	  // make sure that the action hasn't been canceled
	  if (!as_.isActive())
	  {
		  twist_.angular.z = 0;
		  twist_.linear.x = 0;
		  twist_pub_.publish(twist_);
		  return;
	  }

	  if( !has_goal_)
	  {
		  twist_.angular.z = 0;
		  twist_.linear.x = 0;
		  twist_pub_.publish(twist_);
		  return;
	  }
	  // helper variables
	  // ros::Rate r(1);

	  /*
		 y < 400 - move forward
		 x < 300 - turn right
		 x > 340 - turn left
	   */
	  feedback_.success = false;
	  twist_.angular.z = 0;
	  twist_.linear.x = 0; 

	  if ( msg->y >= 170 
		&& msg->x <= 175
		&& msg->x >= 145 )
	  {
		  twist_pub_.publish(twist_);
		  ROS_INFO("%s: Succeeded", action_name_.c_str());
		  // set the action state to succeeded
		  result_.success = true;
		  as_.setSucceeded(result_);
	  }

	  if (msg->y < 170)
	  {
		  ROS_INFO("MOVE FORWARD");
		  twist_.linear.x = getLinVel(msg->y, 170);
	  }

	  if (msg->x < 145)
	  {
		  ROS_INFO("TURN RIGHT");
		  twist_.angular.z = getAngVel(msg->x, 145);
	  }
	  else if (msg->x  > 175)
	  {
		  ROS_INFO("TURN LEFT");
		  twist_.angular.z = -1.0 * getAngVel(msg->x, 175);
	  }

	  twist_pub_.publish(twist_);
	  as_.publishFeedback(feedback_);
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "approach");

  ApproachAction approach("approach");//ros::this_node::getName());
  ros::spin();

  return 0;
}
