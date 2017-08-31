#!/usr/bin/env python

from Tkinter import *
from DroneControl import DroneControl

class App:
	def __init__(self, master):
		droneControl = DroneControl()
		frame = Frame(master)
		frame.pack()
		self.button = Button(frame, text="Return Home", command = droneControl.returnHome)
		self.button.pack()

root = Tk()
app = App(root)
root.mainloop()
