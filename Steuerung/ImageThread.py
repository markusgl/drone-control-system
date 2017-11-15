import threading
import cv2
from cv_bridge import CvBridgeError

class ImageThread(threading.Thread):
	def __init__(self, droneControl, bridge):
		threading.Thread.__init__(self)
		self.droneControl = droneControl
		self.bridge = bridge
	
	#displays the livestream on the GUI
	def run(self, streamFrame):	
		try:
		#decode image
			cv2Img = self.bridge.imgmsg_to_cv2(streamFrame, 'bgr8')
		except CvBridgeError, e:
			print(e)
		
		cv2.startWindowThread()		
		cv2.imshow( "Video Stream", cv2Img)    # load frame into the OpenCV Window
		cv2.waitKey(5)
		self.exit()
