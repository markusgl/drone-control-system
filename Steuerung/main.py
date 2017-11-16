#!/usr/bin/env python

import rospy
import cv2
import time
import rawpy as rp
import numpy as np

from ClassifiyImages import Classify
from sensor_msgs.msg import Image as rosimg
from cv_bridge import CvBridge, CvBridgeError
from DroneControl import DroneControl
from Tkinter import *
from PIL import ImageTk, Image
from multiprocessing import Process
from ImageThread import ImageThread
from ClassifyThread import ClassifyThread
import json
	
class App:
	
	isSleeping = False
	
	def __init__(self, root):
		self.root = root
		self.droneControl = None
		self.bridge = CvBridge()
		self.classifier = Classify("../models/retrained_labels.txt", "../models/retrained_graph.pb", 'DecodeJpeg/contents:0','final_result:0')
		self.frame = Frame(self.root)
		self.frame.pack()
		self.startButton = Button(self.frame, text="Start Drone", command = self.initDrone)
		self.startButton.pack()
		
	def initDrone(self):
		self.droneControl = DroneControl()
		self.homeButton = Button(self.frame, text="Return Home", command = self.writeInFile)
		self.homeButton.pack()
		self.imageThread = ImageThread(self.droneControl, self.bridge)
		self.imageThread.start()
		self.jsonData=[]
		self.classifyThread = ClassifyThread(self.droneControl, self.bridge, self.jsonData, self.classifier)
		self.classifyThread.start()
		self.handleDrone()
		self.initStream()
		
	def forwardImage(self, data):
		dict={}
		try:
			#decode image
			cv2_img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
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
		dict={	'img':rope_image ,
				'pos': ropePosition}
		self.dronecontrol.flyToNextPosition(ropePosition)
		self.jsonData.append(dict)
	

		time.sleep(0.05)

	def handleDrone(self):
		print("Handling Drone")
		rospy.Subscriber('/bebop/image_raw', rosimg, self.classifyThread.forwardImage)
	
	def writeInFile(self):
		self.droneControl.returnHome()
		with open('testPrediction.json', 'a') as outfile:
			outfile.writelines(json.dumps(item)+ '\n' for item in self.jsonData)		
	
	def initStream(self):
		print("Initialize video stream")
		rospy.Subscriber("/bebop/image_raw", rosimg, self.imageThread.streamVideo)
		
def main():
	rospy.init_node('ropeRecognition', anonymous=True)
	root = Tk()
	root.title("Super Drone Application")
	root.geometry("640x640")
	root.configure(background = "grey")
	app = App(root)
	root.mainloop()

if __name__ == "__main__":
	main() 	


