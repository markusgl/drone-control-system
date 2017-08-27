#!/usr/bin/env python

import rospy
import time
import bebop_driver
from std_msgs.msg import Empty
from bebop_msgs.msg import Ardrone3PilotingStateFlyingStateChanged
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged

class DroneControl:

	steeringActive = False
	topReached = True #DEBUG, EIGENTLICH FALSE
	readyToFly = False
	tookOff = False
	lastSteeringCommandY = 0.0
	lastSteeringCommandZ = 0.0
	landingInitialized = False
	isLanding = False
		
	def __init__(self):
		print("Initialize") #DEBUG
		self.emptyMsg = Empty()
		rospy.init_node("droneControl", anonymous=True)
		self.takeoffPub = rospy.Publisher("/bebop/takeoff", Empty, queue_size=10)
		self.readyToFlySub = rospy.Subscriber("/bebop/states/ardrone3/PilotingState/FlyingStateChanged", Ardrone3PilotingStateFlyingStateChanged, self.isReadyToFly)
		self.landingPub = rospy.Publisher("/bebop/land", Empty, queue_size=10)
		self.readyToLandSub = rospy.Subscriber("/bebop/states/ardrone3/PilotingState/AltitudeChanged", Ardrone3PilotingStateAltitudeChanged, self.checkForLanding)
		self.landingSub = rospy.Subscriber("/bebop/states/ardrone3/PilotingState/FlyingStateChanged", Ardrone3PilotingStateFlyingStateChanged, self.isItLanding)
		self.takeoff()

	def takeoff(self):
		self.takeoffSub = rospy.Subscriber("/bebop/takeoff", Empty, self.setTookOff)
		while self.tookOff == False:
			self.takeoffPub.publish(self.emptyMsg)
			print("Try Takeoff") #DEBUG
			time.sleep(3)
		print("Start Takeoff") #DEBUG
		self.takeoffSub.unregister()
		while self.readyToFly == False:
			time.sleep(1)
		self.steeringActive = True
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
		self.flyingPub = rospy.Publisher("/bebop/cmd_vel", geometry_msgs/Twist, queue_size = 15)
		if self.steeringActive:
			self.flyToNextPosition(lastDirection)		

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
				lastSteeringCommandY = -0.75
				lastDirection = 5
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
			print(twistMsg) #DEBUG
		
	def isReadyToFly(self, msg):
		print("Test") #DEBUG
		print(msg.state) #DEUBG
		if msg.state == 1: self.readyToFly = False
		if msg.state == 2: self.readyToFly = True
		else: print("Fehler, Drohne nicht im Startmodus!") #TODO: Sicherheitsregel, falls Drohne nicht im Takeoff
		
	def checkForLanding(self, msg):
		if self.topReached and self.readyToFly:  #readyToFly wieder raus!
						#and msg.altitude <= 1.5
			self.land()
			print("Init Landing Method")

	def land(self):
		self.landingInitialized = True
		while self.isLanding == False:
			self.landingPub.publish(self.emptyMsg)
			time.sleep(1)
			print("Probier mer halt amol zu landen!")
		print("Land")
	
	def isItLanding(self, msg):
		if msg.state == 4: self.isLanding = True
