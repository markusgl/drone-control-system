#!/usr/bin/env python

import rospy
import cv2
import time
import ftplib
import ImageTk

from Steuerung.classify_images import Classify
from sensor_msgs.msg import Image as rosimg
from cv_bridge import CvBridge, CvBridgeError
from Steuerung.DroneControl import DroneControl
from Tkinter import *
from image_stitching.stitcher import Stitcher
	
class App(object):	
	"""
	Class that controls the application and connects the different modules.
	Also loads and controls the GUI
	"""
	
	def __init__(self, root):
		self.count = 0
		self.root = root
		self.drone_control = None
		#self.delete_all_saved_files_on_drone()
		self.bridge = CvBridge()
		self.classifier = Classify("./models/Selbstgebastelt.hdf5")
		self.frame = Frame(self.root)
		self.frame.pack()
				
		self.start_button = Button(self.frame, text="Start Drone", command = self.initDrone)
		self.start_button.pack()
		
	def delete_all_saved_files_on_drone(self):
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
		
	def init_drone(self):
		self.drone_control = DroneControl()
		self.home_button = Button(self.frame, text="Return Home", command = self.start_homecoming)
		self.home_button.pack()
		
		self.handle_drone()
		self.init_stream()
		
	def forward_image(self, data):
		"""
		takes a frame, sends it to the classifier and calls the next steering command
		"""
		try:
			#decode image
			cv2_img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
			rope_position = self.classifier.classify_image(cv2_img)
			
			print(rope_position)
			self.drone_control.flyToNextPosition(rope_position)
			
			time.sleep(0.05)
		except CvBridgeError, e:
			print(e)

	def handle_drone(self):
		print("Handling Drone")
		rospy.Subscriber('classifyTopic', rosimg, self.forward_image)
	
	def start_homecoming(self):
		self.drone_control.return_home()
		
	def stitch_image(self):
		#getting all images from internal media for stitching
		ftp = ftplib.FTP("192.168.42.1:21")
		path = 'internal_000/Bebop_Drone/media'
		ftp.login("anonymous", "")
		ftp.cwd(path)
		ls = ftp.nlst()
		count = len(ls)
		curr = 0
		images = []
		for filename in ls:
			curr += 1
			print 'Processing file {} ... {} of {} ...'.format(filename, curr, count)
			#ftp.retrbinary("RETR " + filename, open(filename, 'wb').write)
			images.append(filename)
		ftp.quit()
		Stitcher.stitch_images(images) #TODO - call stitcher
	
	def init_stream(self):
		print("Initialize video stream")
		rospy.Subscriber("/bebop/image_raw", rosimg, self.stream_video)
		
	def stream_video(self, stream_frame):
		"""
		displays the livestream on the GUI
		"""	
		try:
		#decode image
			cv2_img = self.bridge.imgmsg_to_cv2(stream_frame, 'bgr8')
		except CvBridgeError, e:
			print(e)
		
		cv2.startWindowThread()		
		cv2.imshow( "Video Stream", cv2_img)    # load frame into the OpenCV Window
		cv2.waitKey(5)
		
		
def main():
	rospy.init_node('rope_recognition', anonymous=True)
	root = Tk()
	root.title("Super Drone Application")
	root.geometry("500x360")
	root.configure(background = "white")
	app = App(root)
	root.mainloop()

if __name__ == "__main__":
	main() 	


