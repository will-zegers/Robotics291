#! /usr/bin/env python

import roslib 
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import speedydug_servers.msg
from std_msgs.msg import Int32
import serial
import sys
import thread
import time
import random
import signal

NO_STATE = 0
INIT_STATE = 1
SPIN_STATE = 2
APPROACH_STATE = 3
WALK_STATE = 4
AVOID_OBSTACLE_STATE = 5
PICK_UP_STATE = 6
DROP_STATE = 7
SPIN_LEFT_STATE = 8
SPIN_RIGHT_STATE = 9

NO_MODE = 0
BALL_MODE = 1
BUCKET_MODE = 2

ARM_MODE = "a\n"
US_MODE = "u\n"

ser = None
client = None
pub = None
pub_beacon = None
obstacle = 0
depth = 9999
cur_state = NO_STATE
cur_mode = NO_MODE
inUSMode = None
killUSThread = False
direction = 0


def US_thread():
    global ser
    global killUSThread
    global inUSMode
    global pub
    global obstacle
    global cur_state
    global cur_mode
    while(1):
        if(cur_state == SPIN_STATE and cur_mode == BUCKET_MODE):
            ser.flushInput()
            ser.write("pollb\n")
            line = ser.readline()
            beacon = 0
            if( line.startswith("1") ):	
                print( str(line) + " beacon" )
                beacon = 1
            elif( line.startswith("0") ):	
                print( str(line) + " beacon" )
                beacon = 0
            else:
                print(str(line)+"we are in else state of Beacon")
                beacon = 0
            pub_beacon.publish(Int32(beacon))
        else:
            ser.flushInput()
            ser.write("pollu\n")
            line = ser.readline()
            obstacle = 0
            if( line.startswith("none") ):	
                print( str(line) )
                obstacle = 0
            elif( line.startswith("blue") ):	
                print( str(line) )
                obstacle = 1
            elif( line.startswith("red")):
                print( str(line) )
                obstacle = 2
            elif( line.startswith("both") ):	
                print( str(line) )
                obstacle = 3
            else:
                print(str(line)+"we are in else state of Ultrasonic")
                obstacle = 0
            pub.publish(Int32(obstacle))
            time.sleep(0.25)
            
        #at the end of the thread, check killUSMode flag and if true, halt serial use, set inUSMode to false, and kill thread
        if(killUSThread == True):
            inUSMode = False
            break


###########################################################
# Enter Arm mode by ensuring that US thread is killed and 
# then using serial communication
###########################################################
def enterArmMode():
    global ser
    global killUSThread
    global inUSMode
    
    while(inUSMode == True):    # we need to kill the thread before proceeding
        killUSThread = True
    killUSThread = False        # this place will only be reached if the thread is killed so no further need to kill it
    ser.write(ARM_MODE)


###########################################################
# Enter Ultrasonic mode and spawn a thread which publishes 
# the Ultrasonic readings until this mode is changed to Arm
# mode.
###########################################################
def enterUSMode():
    global ser
    global inUSMode
    ser.write(US_MODE)
    #add code here which spawns a thread updating an US value
    if(inUSMode == False or inUSMode == None):    #if we are not currently in US mode, spawn thread
        inUSMode = True
        #spawn thread here
        thread.start_new_thread( US_thread, ())
    else:
        # the US thread is already spawned
        print("No new US thread needed")

###########################################################
# Callback for distance from ball
###########################################################
def depthCB(data):
    global depth
    # spin() simply keeps python from exiting until this node is stopped
    depth = data.data


###########################################################
# Action Client to approach a ball
# Return : None if canceled
#          True if ball detected
###########################################################
def approach_client():
    global client
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('approach', speedydug_servers.msg.ApproachAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server(timeout=rospy.Duration(30.0))

    # Creates a goal to send to the action server.
    goal = speedydug_servers.msg.ApproachGoal(approach=True)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result(timeout=rospy.Duration(20.0))

    print( "approach result = " + str(client.get_result()) )
    if( client.get_result() == None ):
        client.cancel_goal()
        return None
    # Prints out the result of executing the action
    return client.get_result().success # A FibonacciResult


###########################################################
# Action Client to approach the bucket
# Return : None if canceled
#          True if ball detected
###########################################################
def approach_bucket_client():
    global client
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('approach_bucket', speedydug_servers.msg.ApproachBucketAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server(timeout=rospy.Duration(30.0))

    # Creates a goal to send to the action server.
    goal = speedydug_servers.msg.ApproachBucketGoal(approach=True)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result(timeout=rospy.Duration(20.0))

    print( "approach bucket result = " + str(client.get_result()) )
    if( client.get_result() == None ):
        client.cancel_goal()
        return None
    # Prints out the result of executing the action
    return client.get_result().success # A FibonacciResult
    

###########################################################
# Action Client to turn the base
# int secs : 20 seconds for a 360 deg spin
# bool left : True to spin to the left (Default)
# Return : None if canceled
#          True if ball detected
###########################################################
def spin_client(secs, left=True):
    global client
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('spin', speedydug_servers.msg.SpinAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server(timeout=rospy.Duration(20.0))

    # Creates a goal to send to the action server.
    goal = speedydug_servers.msg.SpinGoal(left)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result(timeout=rospy.Duration(secs))

    print( "spin result = " + str(client.get_result()) )
    if( client.get_result() == None ):
        client.cancel_goal()
        return None

    # Prints out the result of executing the action
    return client.get_result().success # A FibonacciResult


###########################################################
# Action Client to turn the base
# int secs : 20 seconds for a 360 deg spin
# bool left : True to spin to the left (Default)
# Return : None if canceled
#          True if ball detected
###########################################################
def spin_bucket_client(secs, left=True):
    global client
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('spin_bucket', speedydug_servers.msg.SpinBucketAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server(timeout=rospy.Duration(20.0))

    # Creates a goal to send to the action server.
    goal = speedydug_servers.msg.SpinBucketGoal(left)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result(timeout=rospy.Duration(secs))

    print( "spin bucket result = " + str(client.get_result()) )
    if( client.get_result() == None ):
        client.cancel_goal()
        return None

    # Prints out the result of executing the action
    return client.get_result().success # A FibonacciResult
    

###########################################################
# Action Client to move the base forward
# int secs : seconds to move forward before stoping
# Return : None if canceled
#          True if ball detected
#          False if obsticle detected
###########################################################
def walk_client(secs):
    global client
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('walk', speedydug_servers.msg.WalkAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server(timeout=rospy.Duration(20.0))

    # Creates a goal to send to the action server.
    goal = speedydug_servers.msg.WalkGoal(seconds=60)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result(timeout=rospy.Duration(secs))

    print( "walk result = " + str(client.get_result()) )
    if( client.get_result() == None ):
        client.cancel_goal()
        return None

    # Prints out the result of executing the action
    return client.get_result().success # A FibonacciResult


###########################################################
# Action Client to move the base forward
# int secs : seconds to move forward before stoping
# Return : None if canceled
#          True if ball detected
#          False if obsticle detected
###########################################################
def walk_bucket_client(secs):
    global client
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('walk_bucket', speedydug_servers.msg.WalkBucketAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server(timeout=rospy.Duration(20.0))

    # Creates a goal to send to the action server.
    goal = speedydug_servers.msg.WalkBucketGoal(seconds=60)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result(timeout=rospy.Duration(secs))

    print( "walk bucket result = " + str(client.get_result()) )
    if( client.get_result() == None ):
        client.cancel_goal()
        return None

    # Prints out the result of executing the action
    return client.get_result().success # A FibonacciResult
    


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


def machine():

	########## INIT_STATE #################
	global cur_state
	global cur_mode
	global ser
	global pub
	global pub_beacon
	global obstacle
	global direction
	cur_state = INIT_STATE
	cur_mode = BALL_MODE
	
	try:
		ser = serial.Serial("/dev/ttyACM0", 9600)
	except serial.serialutil.SerialException:
		return "Bad Serial"

	# Initializes a rospy node so that the SimpleActionClient can
	# publish and subscribe over ROS.
	rospy.init_node('machine_client_py')
	rospy.Subscriber("/track_point_depth", Int32, depthCB)
	pub = rospy.Publisher("/obstacle", Int32, queue_size=10)
	pub_beacon = rospy.Publisher("/beacon", Int32, queue_size=10)

	# reset the arm to starting position
	# enterArmMode()
	# ser.flushInput()
	# ser.write("reset\n")
	# line = ser.readline()
	# if( line.startswith("g") ):	
	#     print( str(line) )
	# else:
	#     print("Error resetting.")

	# now enter Ultrasonic mode
	time.sleep(3)
	enterUSMode()
	time.sleep(1)
	cur_state = SPIN_STATE
	########## End INIT_STATE #################

	while not rospy.is_shutdown():
		if( cur_state == SPIN_STATE ):
			print( "SPIN_STATE")
			if(cur_mode == BALL_MODE):
				num = random.randint(24, 30)
				result = spin_client(num)
				if( result != None ):
					cur_state = APPROACH_STATE
				else:
					# spun 360 and didn't find a ball. Go straight now
					cur_state = WALK_STATE
			elif(cur_mode == BUCKET_MODE):
				result = None
				if(direction == 0):
					result = spin_bucket_client(30, True)    #left
					direction = 1
				else:
					result = spin_bucket_client(30, False)    #right
					direction = 0
				if( result == True ):    #this means bucket seen. enter approach
					cur_state = APPROACH_STATE
				elif( result == False ):    #this means either ir exit. walk will help decide
					cur_state = WALK_STATE
				else:
					# spun 360 and didn't find a bucket. Go straight now
					cur_state = WALK_STATE
 
		elif( cur_state == SPIN_LEFT_STATE ):
			print( "SPIN_LEFT_STATE")
			if(cur_mode == BALL_MODE):
				result = move_client(1, 6)
				if( result != None ):
					cur_state = SPIN_STATE
				else:
					# spun 90 and didn't find a ball. Go straight now
					cur_state = WALK_STATE
			elif(cur_mode == BUCKET_MODE):
				result = spin_bucket_client(6, True)
				if( result == True ):    #this means bucket seen. enter approach
					cur_state = APPROACH_STATE
				elif( result == False ):    #this means either ir exit. walk will help decide
					cur_state = WALK_STATE
				else:
					# spun 360 and didn't find a bucket. Go straight now
					cur_state = WALK_STATE

		elif( cur_state == SPIN_RIGHT_STATE ):
			print( "SPIN_LEFT_STATE")
			if(cur_mode == BALL_MODE):
				result = move_client(2, 6)
				if( result != None ):
					cur_state = SPIN_STATE
				else:
					# spun 90 and didn't find a ball. Go straight now
					cur_state = WALK_STATE
			elif(cur_mode == BUCKET_MODE):
				result = spin_bucket_client(6, False)
				if( result == True ):    #this means bucket seen. enter approach
					cur_state = APPROACH_STATE
				elif( result == False ):    #this means either ir exit. walk will help decide
					cur_state = WALK_STATE
				else:
					# spun 360 and didn't find a bucket. Go straight now
					cur_state = WALK_STATE

		elif( cur_state == APPROACH_STATE ):
			print( "APPROACH_STATE")
			if(cur_mode == BALL_MODE):
				result = approach_client()
				if( result == True ):
					cur_state = PICK_UP_STATE
				elif( result == False ):
					cur_state = AVOID_OBSTACLE_STATE
				else:
					cur_state = SPIN_STATE
			elif(cur_mode == BUCKET_MODE):
				result = approach_bucket_client()
				if( result == True ):
					cur_state = DROP_STATE
				elif( result == False ):
					cur_state = AVOID_OBSTACLE_STATE
				else:
					cur_state = SPIN_STATE

		elif( cur_state == WALK_STATE ):
			print( "WALK_STATE")
			if(cur_mode == BALL_MODE):
				num = random.randint(5, 7)
				result = walk_client(num)
				if( result == True ):
					cur_state = APPROACH_STATE
				elif( result == False ):
					cur_state = AVOID_OBSTACLE_STATE
				else:
					cur_state = SPIN_STATE
			if(cur_mode == BUCKET_MODE):
				num = random.randint(4, 6)
				result = walk_client(num)
				if( result == True ):
					cur_state = APPROACH_STATE
				elif( result == False ):
					cur_state = AVOID_OBSTACLE_STATE
				else:
					cur_state = SPIN_STATE
		elif( cur_state == AVOID_OBSTACLE_STATE):
			print( "AVOID_OBSTACLE_STATE")
			cur_obs = obstacle
			result = move_client(3, 2)
			num = random.randint(0, 1)
			print( num )
			if(num == 0):
				cur_state = SPIN_LEFT_STATE
			else:
				cur_state = SPIN_RIGHT_STATE

		elif( cur_state == PICK_UP_STATE):
			print( "PICK_OBSTACLE_STATE")
			print( str(depth) + "cm\n" )
			enterArmMode()
			ser.flushInput()
			ser.write(str(depth)+"cm\n")
			line = ser.readline()
			if( line.startswith("g") ):	
				print( str(line) )
				enterUSMode()
				cur_state = SPIN_STATE
				cur_mode = BUCKET_MODE
			elif( line.startswith("b") ):
				print( str(line) )
				enterUSMode()
				result = move_client(3, 2)
				cur_state = SPIN_STATE
			else:
				print( str(line) )
				enterUSMode()
				cur_state = SPIN_STATE

		elif( cur_state == DROP_STATE):
			print( "DROP_STATE" )
			enterArmMode()
			ser.flushInput()
			ser.write("drop\n")
			line = ser.readline()
			if( line.startswith("g") ):	
				print( str(line) )
				enterUSMode()
				cur_state = SPIN_STATE
				cur_mode = BALL_MODE
			elif( line.startswith("b") ):
				print( str(line) )
				enterUSMode()
				cur_state = SPIN_STATE
				cur_mode = BALL_MODE
			else:
				print( str(line) )
				enterUSMode()
				cur_state = SPIN_STATE
				cur_mode = BALL_MODE

		else:
			print("No state defined")
	
def signal_handler(signal, frame):
	global ser
	global client
	print('\nYou pressed Ctrl+C!')
	if (ser != None):
		print('Closing Serial')
		ser.close()
	if (client != None):
		print('Caceling clients')
		client.cancel_goal()
	print('Exit')
	sys.exit(0)

if __name__ == '__main__':
	signal.signal(signal.SIGINT, signal_handler)
	try:
		result = machine()
		print ( result )
	except rospy.ROSInterruptException:
		print "program interrupted before completion"
