#!/usr/bin/env python

import rospy
import cv2
import time
import ftplib

from ClassifiyImages import Classify
from sensor_msgs.msg import Image as rosimg
from cv_bridge import CvBridge, CvBridgeError
from DroneControl import DroneControl
from Tkinter import *
	
class App:	
	def __init__(self, root):
		self.count = 0
		self.root = root
		self.droneControl = None
		#self.deleteAllSavedFilesOnDrone()
		self.bridge = CvBridge()
		self.classifier = Classify("../models/retrained_labels.txt", "../models/retrained_graph.pb", 'DecodeJpeg/contents:0','final_result:0')
		self.frame = Frame(self.root)
		self.frame.pack()
		self.startButton = Button(self.frame, text="Start Drone", command = self.initDrone)
		self.startButton.pack()
		
	def deleteAllSavedFilesOnDrone(self):
		ftp = ftplib.FTP("192.168.42.1:21")
		path = 'internal_000/Bebop_Drone/media'
		ftp.login("anonymous", "")
		ftp.cwd(path)
		#ls of all files on drone internal media
		ls = ftp.nlst()
		count = len(ls)
		curr = 0
		for filename in ls:
			curr += 1
			print 'Deleting file {} ... {} of {} ...'.format(filename, curr, count)
			ftp.delete(filename)
		ftp.quit()
		
	def initDrone(self):
		self.droneControl = DroneControl()
		self.homeButton = Button(self.frame, text="Return Home", command = self.startHomecoming)
		self.homeButton.pack()
		
		self.handleDrone()
		self.initStream()
		
	#takes a frame, sends it to the classifier and calls the next steering command
	def forwardImage(self, data):
		try:
			#decode image
			cv2Img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
			ropePosition = self.classifier.classifyAImage(cv2Img)
			
			print(ropePosition)
			self.dronecontrol.flyToNextPosition(ropePosition)
			
			time.sleep(0.05)
		except CvBridgeError, e:
			print(e)

	def handleDrone(self):
		print("Handling Drone")
		rospy.Subscriber('classifyTopic', rosimg, self.forwardImage)
	
	def startHomecoming(self):
		self.droneControl.returnHome()
		
	def stitchImage(self):
		#getting all images from internal media for stitching
		ftp = ftplib.FTP("192.168.42.1:21")
		path = 'internal_000/Bebop_Drone/media'
		ftp.login("anonymous", "")
		ftp.cwd(path)
		ls = ftp.nlst()
		count = len(ls)
		curr = 0
		for filename in ls:
			curr += 1
			print 'Processing file {} ... {} of {} ...'.format(filename, curr, count)
			ftp.retrbinary("RETR " + filename, open(filename, 'wb').write)
		ftp.quit()
	
	def initStream(self):
		print("Initialize video stream")
		rospy.Subscriber("/bebop/image_raw", rosimg, self.streamVideo)
		
	#displays the livestream on the GUI
	def streamVideo(self, streamFrame):	
		try:
		#decode image
			cv2Img = self.bridge.imgmsg_to_cv2(streamFrame, 'bgr8')
		except CvBridgeError, e:
			print(e)
		
		cv2.startWindowThread()		
		cv2.imshow( "Video Stream", cv2Img)    # load frame into the OpenCV Window
		cv2.waitKey(5)
		
		
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


