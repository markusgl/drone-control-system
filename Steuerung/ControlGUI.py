#!/usr/bin/env python

from Tkinter import *
from DroneControl import DroneControl

#Uses Tkinter to create a simple UI with buttons
class App:
	def __init__(self, master):
		self.frame = Frame(master)
		self.frame.pack()
		self.startButton = Button(self.frame, text="Start Drone", command = self.initDrone)
		self.startButton.pack()
		
	def initDrone(self):
		self.droneControl = DroneControl()
		self.homeButton = Button(self.frame, text="Return Home", command = self.droneControl.returnHome)
		self.homeButton.pack()

root = Tk()
app = App(root)
root.mainloop()
