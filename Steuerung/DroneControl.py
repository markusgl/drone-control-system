#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Empty
from bebop_msgs import Ardrone3PilotingStateFlyingStateChanged

steeringActive = False

def initialize():
	rospy.init_node("droneControl", anonymous=True)
	takeoffPub = rospy.Publisher("/bebop/takeoff", Empty, queue_size=10)
	readyToFlySub = rospy.Subscriber("/bebop/states/ardrone3/PilotingState/FlyingStateChanged", Ardrone3PilotingStateFlyingStateChanged, isReadyToFly)
	
	emptyMsg = Empty()
	takeoff()

def takeoff():
	tookOff = False
	takeoffSub = rospy.Subscriber("/bebop/takeoff", Empty, lambda: tookOff = True)
	while !tookOff
		takeoffPub.publish(emptyMsg)
	takeoffSub.unregister()
	while !isReadyToFly()
		time.sleep(3)
	steeringActive = True

"""
ropePosition is a number between 1 and 5:
1 -> rope is left
2 -> rope is little left
3 -> rope is center
4 -> rope is little right
5 -> rope is right
"""
def fly(ropePosition):
	if steeringActive
		if ropePosition == 1
		elif ropePosition == 2
		elif ropePosition == 3
		elif ropePosition == 4
		elif ropePosition == 5
		
	
def isReadyToFly(msg):
	if msg.state == 1 return False
	elif msg.state == 2 return True
	else print("Fehler, Drohne nicht im Startmodus!") #TODO: Sicherheitsregel, falls Drohne nicht im Takeoff
