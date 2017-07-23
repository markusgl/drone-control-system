#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
import time

from bebop_msgs.msg import Ardrone3PilotingStateFlyingStateChanged
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

tookoff = False
landed = False

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I am going to take off')
    tookoff = True
    rospy.loginfo(tookoff)
    
def callbackLand(data):
	rospy.loginfo(royp.get_caller_id() + 'I am going to land')
	landed = True
	rospy.loginfo(landed)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/bebop/takeoff', Empty, callback)
    rospy.Subscriber('/bebop/land', Empty, callbackLand)

    while not rospy.is_shutdown():
		empt = Empty()
		while tookoff:
			pub2 = rospy.Publisher("/bebop/takeoff", Empty, queue_size=10)
			pub2.publish(empt)
			time.sleep(1)
		time.sleep(15)
		while landed:
			pub2 = rospy.Publisher("/bebop/land", Empty, queue_size=10)
			pub2.publish(empt)
			time.sleep(1)
		break

if __name__ == '__main__':
    listener()
