#!/usr/bin/env python

import rospy
import time
import bebop_driver
import logging
from std_msgs.msg import Empty
from std_msgs.msg import Bool
from bebop_msgs.msg import Ardrone3PilotingStateFlyingStateChanged
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged
from geometry_msgs.msg import Twist

class DroneControl:

	steeringActive = False
	topReached = False
	hovering = False
	tookOff = False
	lastSteeringCommandY = 0.0
	lastSteeringCommandZ = 0.0
	lastDirection = 3
	landingInitialized = False
	isLanding = False
		
	def __init__(self):
		logging.info("Initialize")
		self.emptyMsg = Empty()
		rospy.init_node("droneControl", anonymous=True)
		self.takeoffPub = rospy.Publisher("/bebop/takeoff", Empty, queue_size=10)
		self.readyToFlySub = rospy.Subscriber("/bebop/states/ardrone3/PilotingState/FlyingStateChanged", Ardrone3PilotingStateFlyingStateChanged, self.isReadyToFly)
		self.flyingPub = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size = 15)
		self.landingPub = rospy.Publisher("/bebop/land", Empty, queue_size=10)
		self.readyToLandSub = rospy.Subscriber("/bebop/states/ardrone3/PilotingState/AltitudeChanged", Ardrone3PilotingStateAltitudeChanged, self.checkForLanding)
		self.landingSub = rospy.Subscriber("/bebop/states/ardrone3/PilotingState/FlyingStateChanged", Ardrone3PilotingStateFlyingStateChanged, self.isItLanding)
		self.homecomingPub = rospy.Publisher("bebop/autoflight/navigate_home", Bool, queue_size=10)
		self.takeoff()

	def takeoff(self):
		self.takeoffSub = rospy.Subscriber("/bebop/takeoff", Empty, self.setTookOff)
		while self.tookOff == False:
			self.takeoffPub.publish(self.emptyMsg)
			loggin.info("Try Takeoff")
			time.sleep(3)
		logging.info("Start Takeoff")
		self.takeoffSub.unregister()
		while self.hovering == False:	#waits for the takeoff to be terminated
			time.sleep(1)
		self.steeringActive = True
		self.readyToFlySub.unregister()
		self.hovering = False
		self.fly()

	def setTookOff(self,msg):
		self.tookOff = True

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
	def fly(self):
		if self.steeringActive:
			self.flyToNextPosition(self.lastDirection)		

	# method called when a frame is rendered and the rope position is determined
	def flyToNextPosition(self, ropePosition):
		if self.steeringActive and self.landingInitialized == False:
			twistMsg = Twist()
			self.lastSteeringCommandZ = 0.0
			
			if ropePosition == 1:
				twistMsg.linear.y = 0.75
				self.lastSteeringCommandY = 0.75
				self.lastDirection = 1
			elif ropePosition == 2:
				twistMsg.linear.y = 0.25
				self.lastSteeringCommandY = 0.25
				self.lastDirection = 2
			elif ropePosition == 3:
				self.lastDirection = 3
			elif ropePosition == 4:
				twistMsg.linear.y = -0.25
				self.lastSteeringCommandY = -0.25
				self.lastDirection = 4
			elif ropePosition == 5:
				twistMsg.linear.y = -0.75
				self.lastSteeringCommandY = -0.75
				self.lastDirection = 5
			elif ropePosition == 6:
				self.topReached = True
				twistMsg.linear.y = 0.0
				self.lastSteeringCommandY = 0.0
				twistMsg.linear.z = 0
				self.steeringActive = False
			elif ropePosition == 7:
				twistMsg.linear.y = -1.0 * (lastSteeringCommandY)
				self.lastSteeringCommandY = twistMsg.linear.y
			
			if self.topReached == False and ropePosition != 7:
				twistMsg.linear.z = 0.5
				self.lastSteeringCommandZ = 0.5
			elif ropePosition != 7:
				twistMsg.linear.z = -0.5
				self.lastSteeringCommandZ = -0.5
			else:
				twistMsg.linear.z = 0.0
				
			self.flyingPub.publish(twistMsg)	
			logging.debug("Flying in direction: " + twistMsg) #DEBUG
		
	def isReadyToFly(self, msg):
		if msg.state == 1: self.hovering = False
		if msg.state == 2: self.hovering = True
		
	def checkForLanding(self, msg):
		if self.topReached and msg.altitude <= 1.5:
			self.land()
			logging.info("Init Landing Method")

	def land(self):
		self.landingInitialized = True
		while self.isLanding == False:
			self.landingPub.publish(self.emptyMsg)
			time.sleep(1)
			logging.info("Probier mer halt amol zu landen!")
		logging.info("Land")
	
	def isItLanding(self, msg):
		if msg.state == 4: self.isLanding = True
		
	#emergency method forces the drone to come home	
	def returnHome(self):
		self.landAfterHomecomingSub = rospy.Subscriber("/bebop/states/ardrone3/PilotingState/FlyingStateChanged", Ardrone3PilotingStateFlyingStateChanged, self.landAtHome)
		boolMsg = Bool(True)
		self.flyingPub = None
		self.homecomingPub.publish(boolMsg)
		logging.info("Returning home")
		
	#landing after returnHome-Method	
	def landAtHome(self, msg):
		if msg.state==2: self.land()
