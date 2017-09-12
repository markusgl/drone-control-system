#!/usr/bin/env python

import rospy
import cv2
import time
import ControlGUI

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from DroneControl import DroneControl
from Tkinter import *
#import Inception as inception

bridge = CvBridge()

def forwardImage(data):
	global count, app
	print("Received an image!")
	try:
		cv2_img = bridge.imgmsg_to_cv2(data, data.encoding)
	except CvBridgeError, e:
		print(e)
	else: 
		count = count + 1
		rope_image = 'image' + str(count) + '.jpeg'
		cv2.imwrite(rope_image, cv2_img)
	#ropePosition = inception.getRopePosition(rope_image)
	app.dronecontrol.flyToNextPosition(ropePosition)
	time.sleep(5)

def handleDrone():
	global app
	rospy.init_node('ropeRecognition', anonymous=True)
	#if drone is started:
	while app.dronecontrol.isReadyToFly == False:
		time.sleep(1)
	rospy.Subscriber('/bebop/image_raw', Image, callback)
	rospy.spin() #keep alive
    
def main():
	global count, app
	count = 0
	root = Tk()
	app = App(root)
	root.mainloop() #starting drone with return home buttone
	handleDrone()
	
	
class App:
	def __init__(self, master):
		self.frame = Frame(master)
		self.frame.pack()
		self.startButton = Button(self.frame, text="Start Drone", command = self.initDrone)
		self.startButton.pack()
		
	def initDrone(self):
		self.dronecontrol = DroneControl()
		self.homeButton = Button(self.frame, text="Return Home", command = self.droneControl.returnHome)
		self.homeButton.pack()

main()


