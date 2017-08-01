#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Empty
from bebop_msgs import Ardrone3PilotingStateFlyingStateChanged

flyingPub
steeringActive = False
topReached = False
lastSteeringCommand = 0.0

def initialize():
	rospy.init_node("droneControl", anonymous=True)
	takeoffPub = rospy.Publisher("/bebop/takeoff", Empty, queue_size=10)
	readyToFlySub = rospy.Subscriber("/bebop/states/ardrone3/PilotingState/FlyingStateChanged", Ardrone3PilotingStateFlyingStateChanged, isReadyToFly)
	
	takeoff()

def takeoff():
	tookOff = False
	emptyMsg = Empty()
	takeoffSub = rospy.Subscriber("/bebop/takeoff", Empty, lambda: tookOff = True)
	while !tookOff
		takeoffPub.publish(emptyMsg)
	takeoffSub.unregister()
	while !isReadyToFly()
		time.sleep(3)
	steeringActive = True
	fly()

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
	flyingPub = rospy.Publisher("/bebop/cmd_vel", geometry_msgs/Twist, queue_size = 15)
	if steeringActive
		flyToNextPosition(lastDirection)		

def flyToNextPosition(ropePosition):
	if steeringActive
		twistMsg = Twist()
		
		if ropePosition == 1
			twistMsg.linear.y = 0.75
			lastDirection = 1
		elif ropePosition == 2
			twistMsg.linear.y = 0.25
			lastDirection = 2
		elif ropePosition == 3
			lastDirection = 3
		elif ropePosition == 4
			twistMsg.linear.y = -0.25
			lastDirection = 4
		elif ropePosition == 5
			twistMsg.linear.y = -0.75
			lastDirection = 5
		elif ropePosition == 6
			topReached = True
			twistMsg.linear.y = 0
			twistMsg.linear.z = 0
			steeringActive = False
		elif ropePosition == 7
			twistMsg.linear.y = -1.0 * (lastSteeringCommand)
		
		if !topReached && ropePosition != 7
			twistMsg.linear.z = 0.5
		elif ropePosition != 7
			twistMsg.linear.z = -0.5
		else
			twistMsg.linear.z = 0
			
		flyingPub.publish(twistMsg)	
	
def isReadyToFly(msg):
	if msg.state == 1 return False
	elif msg.state == 2 return True
	else print("Fehler, Drohne nicht im Startmodus!") #TODO: Sicherheitsregel, falls Drohne nicht im Takeoff
