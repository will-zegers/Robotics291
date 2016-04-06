#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <speedydug_servers/SpinBucketAction.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>

#define ANG_VEL 0.3141592653

class SpinBucketAction
{
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<speedydug_servers::SpinBucketAction> as_;
  std::string action_name_;
  // create messages that are used to published feedback/result
  speedydug_servers::SpinBucketFeedback feedback_;
  speedydug_servers::SpinBucketResult result_;
  ros::Subscriber point_sub_;
  ros::Subscriber ir_sub_;
  ros::Publisher twist_pub_;
  geometry_msgs::Twist twist_;
  bool bucket_detected_;
  bool ir_detected_;
  bool entered_ir;

public:

  SpinBucketAction(std::string name) :
	  as_(nh_, name, boost::bind(&SpinBucketAction::executeCB, this, _1), false),
	  action_name_(name)
  {
	// register the goal and feeback callbacks
	point_sub_ = nh_.subscribe("/track_bucket_point_avg", 1, &SpinBucketAction::trackBucketPointCB, this);
	ir_sub_ = nh_.subscribe("/beacon", 1, &SpinBucketAction::trackIRCB, this);
	twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	bucket_detected_=false;
  	ir_detected_=false;
    entered_ir = false;

    as_.start();
  }

  ~SpinBucketAction(void)
  {
  }


  void executeCB(const speedydug_servers::SpinBucketGoalConstPtr &goal)
  {
	  // helper variables
	  bucket_detected_ = false;
      ir_detected_ = false;
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

	  while(!bucket_detected_ && !ir_detected_ )
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

      if(bucket_detected_ )
      {
          twist_.angular.z = 0;
          twist_.linear.x = 0;
          twist_pub_.publish(twist_);
          result_.success = true;
          ROS_INFO("%s: Succeeded", action_name_.c_str());
          // set the action state to succeeded
          as_.setSucceeded(result_);
      } else if(ir_detected_) {
          twist_.angular.z = 0;
          twist_.linear.x = 0;
          twist_pub_.publish(twist_);
          result_.success = false;
          ROS_INFO("%s: Succeeded", action_name_.c_str());
          // set the action state to succeeded
          as_.setSucceeded(result_);
      }
  }

  void trackBucketPointCB(const geometry_msgs::Point::ConstPtr& msg)
  {
	  // make sure that the action hasn't been canceled
	  if (!as_.isActive())
		  return;
	  bucket_detected_ = true;
  }

  void trackIRCB(const std_msgs::Int32::ConstPtr& msg)
  {
	  // make sure that the action hasn't been canceled
	  if (!as_.isActive())
		  return;
      if ( msg->data == 1 )
      {
          entered_ir = true;
      }
      if (entered_ir && (msg->data == 0))
      {
          ir_detected_ = true;
          entered_ir = false;
      }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "spin_bucket");

  SpinBucketAction spin_bucket("spin_bucket");//ros::this_node::getName());
  ros::spin();

  return 0;
}
