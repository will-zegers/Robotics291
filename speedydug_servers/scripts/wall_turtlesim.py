#! /usr/bin/env python
import rospy
import math
from std_msgs.msg import Int32
from std_msgs.msg import Float64
import turtlesim.msg

wall_distance = 32
PI = 3.141592653

def handle_turtle_pose(msg, pub):
	global wall_distance
	wall_distance = 32
	obstacle = 0;

	x = msg.x
	y = msg.y
	if( msg.theta < 0 ):
		theta = msg.theta + 2*PI
	else:
		theta = msg.theta

	if ( theta >= PI/4 and theta < 3*PI/4):
		wall_distance = math.fabs(11 - y)	
	elif ( theta >= 3*PI/4 and theta < 5*PI/4):
		wall_distance = math.fabs(0 - x)	
	elif ( theta >= 5*PI/4 and theta < 7*PI/4):
		wall_distance = math.fabs(0 - y)	
	else: 	
		wall_distance = math.fabs(11 - x)

	if( wall_distance < 1 ):
		obstacle = 1

	pub.publish(Int32(obstacle))
	 

if __name__ == '__main__':

	rospy.init_node('node_name')
	pub = rospy.Publisher('obstacle', Int32, queue_size=10)
	rospy.Subscriber('/turtle1/pose',
			turtlesim.msg.Pose,
			handle_turtle_pose,
			pub)
	r = rospy.Rate(2) # 10hz

	while not rospy.is_shutdown():
		r.sleep()

