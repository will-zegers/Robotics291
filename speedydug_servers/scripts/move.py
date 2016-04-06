#! /usr/bin/env python

import roslib 
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import speedydug_servers.msg
from std_msgs.msg import Int32


###########################################################
# Action Client to move unconditionally
# Return : True if Action success
#          False if requested action is not availiable
###########################################################
def move_client(move, secs=1):
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('move', speedydug_servers.msg.MoveAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server(timeout=rospy.Duration(20.0))

    # Creates a goal to send to the action server.
    goal = speedydug_servers.msg.MoveGoal(move=move)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result(timeout=rospy.Duration(secs))

    print( "move result = " + str(client.get_result()) )
    if( client.get_result() == None ):
        client.cancel_goal()
    # Prints out the result of executing the action
    return client.get_result() # A FibonacciResult


if __name__ == '__main__':
	try:
		rospy.init_node('machine_client_py')
		result = move_client(0, 1)
		print ( result )
		result = move_client(1, 7)
		print ( result )
		result = move_client(2, 7)
		print ( result )
		result = move_client(3, 1)
		print ( result )
		result = move_client(4, 7)
		print ( result )
	except rospy.ROSInterruptException:
		print "program interrupted before completion"
