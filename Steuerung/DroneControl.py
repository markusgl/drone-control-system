#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Empty
from std_msgs.msg import Bool
from bebop_msgs.msg import Ardrone3PilotingStateFlyingStateChanged
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged
from geometry_msgs.msg import Twist

class DroneControl(object):
	"""
	Class for controlling the drone. Includes all relevant commands and methods
	for flying.
	"""

	steering_active = False
	top_reached = False
	hovering = False
	took_off = False
	last_steering_command_y = 0.0
	last_steering_command_z = 0.0
	current_altitude = 0.0
	last_altitude = 0.0
	snapshot_distance = 1.0	#every x meters a snapshot is taken when the rope is in the center
	last_direction = 3
	landing_initialized = False
	is_landing = False
		
	def __init__(self):
		print("Initialize")
		self.empty_msg = Empty()
		self.takeoff_pub = rospy.Publisher("/bebop/takeoff", Empty, queue_size = 10) 						#Publisher for takeoff
		self.ready_to_fly_sub = rospy.Subscriber("/bebop/states/ardrone3/PilotingState/FlyingStateChanged",
		 Ardrone3PilotingStateFlyingStateChanged, self.is_ready_to_fly)										#Subscriber that checks if drone is ready to fly
		self.flying_pub = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size = 15)							#Publisher for flying
		self.landing_pub = rospy.Publisher("/bebop/land", Empty, queue_size = 10)							#Publisher for landing
		self.ready_to_land_sub = rospy.Subscriber("/bebop/states/ardrone3/PilotingState/AltitudeChanged",
		 Ardrone3PilotingStateAltitudeChanged, self.check_for_landing)										#Subscriber that checks is landing can be started
		self.check_altitude_sub = rospy.Subscriber("/bebop/states/ardrone3/PilotingState/AltitudeChanged",
		 Ardrone3PilotingStateAltitudeChanged, self.set_altitude)											#Subscriber that sets the current altitude value
		self.landing_sub = rospy.Subscriber("/bebop/states/ardrone3/PilotingState/FlyingStateChanged",
		 Ardrone3PilotingStateFlyingStateChanged, self.set_if_landing)										#Subscriber that checks if the drone is landing
		self.homecoming_pub = rospy.Publisher("bebop/autoflight/navigate_home", Bool, queue_size = 10)		#Publisher for the return home mission
		self.snapshot_pub = rospy.Publisher("/bebop/snapshot", Empty, queue_size = 10)						#Publisher for taking Pictures
		self.takeoff()

	def takeoff(self):
		self.takeoff_sub = rospy.Subscriber("/bebop/takeoff", Empty, self.set_took_off)
		while not self.took_off:
			self.takeoff_pub.publish(self.empty_msg)
			print("Try Takeoff")
			time.sleep(3)
		print("Start Takeoff")
		self.takeoff_sub.unregister()
		while not self.hovering:	#waits for the takeoff to be terminated
			time.sleep(1)
		self.steering_active = True
		self.ready_to_fly_sub.unregister()
		self.hovering = False
		self.fly()

	def set_took_off(self,msg):
		self.took_off = True

	"""
	ropePosition is a number between 1 and 5:
	0 -> rope is left
	1 -> rope is little left
	2 -> rope is center
	3 -> rope is little right
	4 -> rope is right
	5 -> Top is reached
	-1 -> No rope detected
	"""
	def fly(self):
		if self.steering_active:
			self.fly_to_next_position(self.last_direction)		

	def fly_to_next_position(self, rope_position):
		"""
		- Method called when a frame is rendered and the rope position is determined
		- sends the next flying-command via flying-publisher
		"""
		if self.steering_active and not self.landing_initialized:
			twist_msg = Twist()
			self.last_steering_command_z = 0.0
			
			if rope_position == 0:
				twist_msg.linear.y = 0.75
				self.last_steering_command_y = 0.75
				self.last_direction = 1
			elif rope_position == 1:
				twist_msg.linear.y = 0.25
				self.last_steering_command_y = 0.25
				self.last_direction = 2
			elif rope_position == 2:
				self.last_direction = 3
			elif rope_position == 3:
				twist_msg.linear.y = -0.25
				self.last_steering_command_y = -0.25
				self.last_direction = 4
			elif rope_position == 4:
				twist_msg.linear.y = -0.75
				self.last_steering_command_y = -0.75
				self.last_direction = 5
			elif rope_position == 5:
				self.top_reached = True
				twist_msg.linear.y = 0.0
				self.last_steering_command_y = 0.0
				twist_msg.linear.z = 0
				self.steering_active = False
			elif rope_position == -1:
				twist_msg.linear.y = -1.0 * (self.last_steering_command_y)
				self.last_steering_command_y = twist_msg.linear.y
			
			if not self.top_reached and rope_position != -1:
				if rope_position == 2 and self.current_altitude - self.last_altitude >= self.snapshot_distance:
					self.snapshot_pub.publish(self.empty_msg)	#if the rope is in the center and the drone is on the way up take a pic
				twist_msg.linear.z = 0.5
				self.last_steering_command_z = 0.5
			elif rope_position != -1:
				twist_msg.linear.z = -0.5
				self.last_steering_command_z = -0.5
			else:
				twist_msg.linear.z = 0.0
				
			self.flying_pub.publish(twist_msg)	
			print(twist_msg) #DEBUG
		
	def is_ready_to_fly(self, msg):
		if msg.state == 1: self.hovering = False
		if msg.state == 2: self.hovering = True
		
	def check_for_landing(self, msg):
		if self.top_reached and msg.altitude <= 1.5:
			self.land()
			print("Init Landing Method")
			
	def set_altitude(self, msg):
		self.last_altitude = self.current_altitude
		self.current_altitude = msg.altitude

	def land(self):
		self.landing_initialized = True
		while not self.is_landing:
			self.landing_pub.publish(self.empty_msg)
			time.sleep(1)
			print("Probier mer halt amol zu landen!")
		print("Land")
	
	def set_if_landing(self, msg):
		if msg.state == 4: self.is_landing = True
		
	def return_home(self):
		"""
		- lets the drone fly back to the start position
		- called when "return home" button is clicked
		"""
		self.land_after_homecoming_sub = rospy.Subscriber("/bebop/states/ardrone3/PilotingState/FlyingStateChanged",
		 Ardrone3PilotingStateFlyingStateChanged, self.land_at_home)
		bool_msg = Bool(True)
		self.flying_pub = None
		self.homecoming_pub.publish(bool_msg)
		print("Returning home")
			
	def land_at_home(self, msg):
		"""
		landing after returnHome-Method
		"""
		if msg.state==2: self.land()
