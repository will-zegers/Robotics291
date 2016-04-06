#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <speedydug_servers/MoveAction.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>


#define LIN_VEL 0.05
#define ANG_VEL 0.3141592653

#define STAY 0
#define TURN_LEFT 1
#define TURN_RIGHT 2
#define BACK_UP 3

#define SERVICE_TIME 55 //roughly 2 full spins 

class MoveAction
{
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<speedydug_servers::MoveAction> as_; 
  std::string action_name_;
  // create messages that are used to published feedback/result
  speedydug_servers::MoveFeedback feedback_;
  speedydug_servers::MoveResult result_;
  ros::Publisher twist_pub_;
  geometry_msgs::Twist twist_;

  
public:

  MoveAction(std::string name) :
	  as_(nh_, name, boost::bind(&MoveAction::executeCB, this, _1), false),
	  action_name_(name)
  {
	// register the goal and feeback callbacks
	twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    as_.start();
  }

  ~MoveAction(void)
  {
  }

  void executeCB(const speedydug_servers::MoveGoalConstPtr &goal)
  {
      feedback_.success = false;
	  float spin_rate = 0.05;
	  int cycles = SERVICE_TIME / spin_rate;
	  int counter = 0;
	  ros::Duration d(spin_rate);

	  while(counter < cycles)
	  {
		  if (!as_.isActive() || !ros::ok() )
		  {
			  twist_.angular.z = 0;
			  twist_.linear.x = 0;
			  twist_pub_.publish(twist_);
			  return;
		  }
		  if (as_.isPreemptRequested() || !ros::ok())
		  {
			  twist_.angular.z = 0;
			  twist_.linear.x = 0;
			  twist_pub_.publish(twist_);
			  ROS_INFO("%s: Preempted", action_name_.c_str());
			  // set the action state to preempted
			  as_.setPreempted();
			  return;
		  }

		  if( goal->move == STAY ){ 
			  twist_.angular.z = 0.0;
			  twist_.linear.x = 0.0;
			  twist_pub_.publish(twist_);
			  d.sleep();
		  }
		  else if( goal->move == TURN_LEFT ){ 
			  twist_.angular.z = ANG_VEL;
			  twist_.linear.x = 0.0 ;
			  twist_pub_.publish(twist_);
			  d.sleep();
		  }
		  else if( goal->move == TURN_RIGHT ){ 
			  twist_.angular.z = -ANG_VEL;
			  twist_.linear.x = 0.0;
			  twist_pub_.publish(twist_);
			  d.sleep();
		  }
		  else if( goal->move == BACK_UP ){
			  twist_.angular.z = 0.0;
			  twist_.linear.x = -LIN_VEL;
			  twist_pub_.publish(twist_);
			  d.sleep();
		  }
		  else{
			  result_.success = false;
			  ROS_INFO("%s: Movement not supported", action_name_.c_str());
			  // set the action state to succeeded
			  as_.setSucceeded(result_);
		  }
		  counter++;
	  }
	
	  twist_.angular.z = 0.0;
	  twist_.linear.x = 0.0;
	  twist_pub_.publish(twist_);

	  result_.success = true;
	  ROS_INFO("%s: Movement completed", action_name_.c_str());
	  // set the action state to succeeded
	  as_.setSucceeded(result_);
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move");

  MoveAction move("move");//ros::this_node::getName());
  ros::spin();

  return 0;
}
