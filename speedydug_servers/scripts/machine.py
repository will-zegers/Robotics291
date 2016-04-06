#! /usr/bin/env python

import roslib; roslib.load_manifest('actionlib_tutorials')
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import speedydug_servers.msg

def approach_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('approach', speedydug_servers.msg.ApproachAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server(timeout=rospy.Duration(20.0))

    # Creates a goal to send to the action server.
    goal = speedydug_servers.msg.ApproachGoal(approach=True)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result(timeout=rospy.Duration(20.0))

    print( "approach result = " + str(client.get_result()) )
    if( client.get_result() == None ):
        client.cancel_goal()
    # Prints out the result of executing the action
    return client.get_result() # A FibonacciResult
	

def spin_client(secs):
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('spin', speedydug_servers.msg.SpinAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server(timeout=rospy.Duration(20.0))

    # Creates a goal to send to the action server.
    goal = speedydug_servers.msg.SpinGoal(seconds=60)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result(timeout=rospy.Duration(secs))

    print( "spin result = " + str(client.get_result()) )
    if( client.get_result() == None ):
        client.cancel_goal()

    # Prints out the result of executing the action
    return client.get_result() # A FibonacciResult


if __name__ == '__main__':
	try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
		rospy.init_node('machine_client_py')

		while not rospy.is_shutdown():
			if( spin_client(10.0) != None ):
				if( approach_client() != None ):
					print( "Success" )
					break;
			else:
				print( "call random walk server" )
				
	except rospy.ROSInterruptException:
		print "program interrupted before completion"
