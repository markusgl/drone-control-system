import threading
import cv2
import time
from cv_bridge import CvBridgeError

class ClassifyThread(threading.Thread):
	def __init__(self, droneControl, bridge, jsonData, classifier):
		threading.Thread.__init__(self)
		self.droneControl = droneControl
		self.bridge = bridge
		self.jsonData = jsonData
		self.classifier = classifier
		self.count = 0
	
	#displays the livestream on the GUI
	def run(self, data):	
		dict={}
		try:
			#decode image
			cv2Img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
		except CvBridgeError, e:
			print(e)
		else:
			#save image
			self.count = self.count + 1
			ropeImage = 'image' + str(self.count) + '.jpeg'
			print("Received an image: " + ropeImage)
		
			cv2.imwrite(ropeImage, cv2Img)
		ropePosition = self.classifier.classifyAImage(ropeImage)
		print(ropePosition)
		dict={	'img':ropeImage ,
				'pos': ropePosition}
		self.droneControl.flyToNextPosition(ropePosition)
		self.jsonData.append(dict)
	
		time.sleep(0.05)
		self.exit()
