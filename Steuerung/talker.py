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

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import roslib
import rospy
import time

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty	
from std_msgs.msg import String

tookoff = False


def talker():
	empt = Empty() #Empty instance for publishing takeoff/land
	soFly = Twist() #Twist instance for publishing flying data
	pubinit = rospy.Publisher('chatter', String, queue_size=1)

	rospy.init_node('cmd_vel_talker', anonymous=True)

	hello_str = "1. gonna publish some things..."
	rospy.loginfo(hello_str)
	pubinit.publish(hello_str)

	pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
	
	while(!tookoff):
		pub2 = rospy.Publisher("/bebop/takeoff", Empty, queue_size=10)
		pub2.publish(empt)
		time.sleep(1)
	
	tak = "published takeoff"
	rospy.loginfo(tak)
	pubinit.publish(tak)
    
	soFly.linear.x = +0.5 #flying right
	
	soFly.linear.y = +0.5 #flying forward
	pub.publish(soFly)
	time.sleep(10)
	
	mov = "published move operations"
	rospy.loginfo(mov)
	pubinit.publish(mov)

	pub2 = rospy.Publisher("/bebop/land", Empty, queue_size=10)
	pub2.publish(empt)
	time.sleep(10)
	
	tak = "published land"
	rospy.loginfo(tak)
	pubinit.publish(tak)

if __name__ == '__main__':
	try:
		talker()
		listener()
	except rospy.ROSInterruptException:
		print 'Exception!'
