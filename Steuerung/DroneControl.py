#!/usr/bin/env python

import rospy
import time
import bebop_driver
from std_msgs.msg import Empty
from bebop_msgs.msg import Ardrone3PilotingStateFlyingStateChanged
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged

steeringActive = False
topReached = False
lastSteeringCommandY = 0.0
lastSteeringCommandZ = 0.0
landingInitialized = False

def initialize():
	print("Hallo")
	global emptyMsg
	emptyMsg = Empty()
	rospy.init_node("droneControl", anonymous=True)
	global takeoffPub
	takeoffPub = rospy.Publisher("/bebop/takeoff", Empty, queue_size=10)
	readyToFlySub = rospy.Subscriber("/bebop/states/ardrone3/PilotingState/FlyingStateChanged", Ardrone3PilotingStateFlyingStateChanged, isReadyToFly)
	global landingPub
	landingPub = rospy.Publisher("/bebop/land", Empty, queue_size=10)
	readyToLandSub = rospy.Subscriber("/bebop/states/ardrone3/PilotingState/AltitudeChanged", Ardrone3PilotingStateAltitudeChanged, checkForLanding)
	
	takeoff()

def takeoff():
	tookOff = False
	takeoffSub = rospy.Subscriber("/bebop/takeoff", Empty, setTookOff)
	while tookOff == False:
		takeoffPub.publish(emptyMsg)
		print("Probiere Takeoff") #DEBUG
	print("Takeoff") #DEBUG
	takeoffSub.unregister()
	while isReadyToFly() == False:
		time.sleep(3)
	steeringActive = True
	fly()

def setTookOff(tookOff):
	tookOff = True

"""
ropePosition is a number between 1 and 5:
1 -> rope is left
2 -> rope is little left
3 -> rope is center
4 -> rope is little right
5 -> rope is right
6 -> Top is reached
7 -> No rope detected
"""
def fly():
	global flyingPub
	flyingPub = rospy.Publisher("/bebop/cmd_vel", geometry_msgs/Twist, queue_size = 15)
	if steeringActive:
		flyToNextPosition(lastDirection)		

def flyToNextPosition(ropePosition):
	if steeringActive and landingInitialized == False:
		twistMsg = Twist()
		lastSteeringCommandZ = 0.0
		
		if ropePosition == 1:
			twistMsg.linear.y = 0.75
			lastSteeringCommandY = 0.75
			lastDirection = 1
		elif ropePosition == 2:
			twistMsg.linear.y = 0.25
			lastSteeringCommandY = 0.25
			lastDirection = 2
		elif ropePosition == 3:
			lastDirection = 3
		elif ropePosition == 4:
			twistMsg.linear.y = -0.25
			lastSteeringCommandY = -0.25
			lastDirection = 4
		elif ropePosition == 5:
			twistMsg.linear.y = -0.75
			lastSteeringCommandY = -0.75
			lastDirection = 5
		elif ropePosition == 6:
			topReached = True
			twistMsg.linear.y = 0.0
			lastSteeringCommandY = 0.0
			twistMsg.linear.z = 0
			steeringActive = False
		elif ropePosition == 7:
			twistMsg.linear.y = -1.0 * (lastSteeringCommandY)
			lastSteeringCommandY = twistMsg.linear.y
		
		if topReached == False and ropePosition != 7:
			twistMsg.linear.z = 0.5
			lastSteeringCommandZ = 0.5
		elif ropePosition != 7:
			twistMsg.linear.z = -0.5
			lastSteeringCommandZ = -0.5
		else:
			twistMsg.linear.z = 0.0
			
		flyingPub.publish(twistMsg)	
		print(twistMsg) #DEBUG
	
def isReadyToFly(msg):
	if msg.state == 1: return False
	elif msg.state == 2: return True
	else: print("Fehler, Drohne nicht im Startmodus!") #TODO: Sicherheitsregel, falls Drohne nicht im Takeoff
	
def checkForLanding(msg):
	if topReached and msg.altitude <= 1.5:
		land()

def land():
	landingInitialized = True
	landingPub.publish(emptyMsg)
	print("Land")
	
initialize()

