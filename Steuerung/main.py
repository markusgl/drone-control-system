#!/usr/bin/env python

import rospy
import cv2
import time
import Rnn as rnn

from sensor_msgs.msg import Image as rosimg
from cv_bridge import CvBridge, CvBridgeError
from DroneControl import DroneControl
from Tkinter import *
from multiprocessing import Process


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
	ropePosition = rnn.getRopePosition(rope_image)
	print(ropePosition)
	app.dronecontrol.flyToNextPosition(ropePosition)
	time.sleep(5)

def handleDrone():
	global app
	#if drone is started:
	print("Handling Drone")
	rospy.Subscriber('/bebop/image_raw', rosimg, forwardImage)
	rospy.spin() #keep alive
    
def main():
	global count, app
	count = 0
	rospy.init_node('ropeRecognition', anonymous=True)
	root = Tk()
	app = App(root)
	rnn.load_graph() # load tf model once at startup
	#TODO: Parallelisieren!
	controlDrone = Process(target=root.mainloop)
	controlDrone.start()
	time.sleep(10)
	image_forwarding = Process(target=handleDrone)
	image_forwarding.start()
		
	
class App:
	def __init__(self, master):
		self.flying_started = False
		self.dronecontrol = None
		self.frame = Frame(master)
		self.frame.pack()
		self.startButton = Button(self.frame, text="Start Drone", command = self.initDrone)
		self.startButton.pack()
		
	def initDrone(self):
		self.dronecontrol = DroneControl()
		self.flying_started = True
		self.homeButton = Button(self.frame, text="Return Home", command = self.dronecontrol.returnHome)
		self.homeButton.pack()

if __name__ == "__main__":
	main() 	


