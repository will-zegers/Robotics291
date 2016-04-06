#! /usr/bin/env python

import roslib 
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import speedydug_servers.msg
from std_msgs.msg import Int32
from std_msgs.msg import Float64
import sys

cur_state = 0
WALK_STATE = 21

def spin_client(secs, left):
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('spin', speedydug_servers.msg.SpinAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server(timeout=rospy.Duration(20.0))

    # Creates a goal to send to the action server.
    goal = speedydug_servers.msg.SpinGoal(left=left)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result(timeout=rospy.Duration(secs))

    print( "spin result = " + str(client.get_result()) )
    if( client.get_result() == None ):
        client.cancel_goal()

    # Prints out the result of executing the action
    return client.get_result() # A FibonacciResult

def walk_client(secs):
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('walk', speedydug_servers.msg.WalkAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server(timeout=rospy.Duration(20.0))

    # Creates a goal to send to the action server.
    goal = speedydug_servers.msg.WalkGoal(seconds=secs)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result(timeout=rospy.Duration(secs))

    print( "walk result = " + str(client.get_result()) )
    if( client.get_result() == None ):
        client.cancel_goal()

    # Prints out the result of executing the action
    return client.get_result() # A FibonacciResult


def search():
	# Initializes a rospy node so that the SimpleActionClient can
	# publish and subscribe over ROS.
	rospy.init_node('machine_client_py')
	
	left = False
	cur_state = WALK_STATE
	while not rospy.is_shutdown(): 
		if( cur_state == WALK_STATE ):
			result = walk_client(100) #walk ~ 20 meters
			if( result == None ):
				print("calling walk")	
			else:
				if( result.success == False ):
					spin_client(5, left)
				 	walk_result = walk_client(10)
					if (walk_result != None):
						if( walk_result.success == False ):
							spin_client(5, left)
					spin_result = spin_client(5, left)
					left = not left	
				if( spin_result != None ):
					return True
		

if __name__ == '__main__':

	try:
		result = search()
		print ( result )
	except rospy.ROSInterruptException:
		print "program interrupted before completion"
