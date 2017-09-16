#!/usr/bin/env python

import rospy
import cv2
import time

from ClassifiyImages import Classify
from sensor_msgs.msg import Image as rosimg
from cv_bridge import CvBridge, CvBridgeError
from DroneControl import DroneControl
from Tkinter import *
from multiprocessing import Process

	
class App:
	def __init__(self, master):
		self.dronecontrol = None
		self.bridge = CvBridge()
		self.count = 0
		self.classifier = Classify("models/retrained_labels.txt", "models/retrained_graph.pb", 'DecodeJpeg/contents:0','final_result:0')
		self.frame = Frame(master)
		self.frame.pack()
		self.startButton = Button(self.frame, text="Start Drone", command = self.initDrone)
		self.startButton.pack()
		
	def initDrone(self):
		self.dronecontrol = DroneControl()
		self.homeButton = Button(self.frame, text="Return Home", command = self.dronecontrol.returnHome)
		self.homeButton.pack()
		self.handleDrone()
		
	def forwardImage(self, data):
		try:
			#decode image
			cv2_img = self.bridge.imgmsg_to_cv2(data, 'rgb8')
		except CvBridgeError, e:
			print(e)
		else:
			#save image
			self.count = self.count + 1
			rope_image = 'image' + str(self.count) + '.jpeg'
			print("Received an image: " + rope_image)
			cv2.imwrite(rope_image, cv2_img)
		ropePosition = self.classifier.classifyAImage(rope_image)
		print(ropePosition)
		self.dronecontrol.flyToNextPosition(ropePosition)
		time.sleep(2)

	def handleDrone(self):
		print("Handling Drone")
		rospy.Subscriber('/bebop/image_raw', rosimg, self.forwardImage)
				
		
def main():
	rospy.init_node('ropeRecognition', anonymous=True)
	root = Tk()
	app = App(root)
	root.mainloop()

if __name__ == "__main__":
	main() 	


