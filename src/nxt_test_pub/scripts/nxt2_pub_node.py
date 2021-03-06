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

import roslib; 

import rospy
from std_msgs.msg import UInt16
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import nxt
import nxt.locator
from nxt.sensor import *
import time

global b 
b = nxt.locator.find_one_brick(name='NXT2')

light=Light(b, PORT_2)
light.set_illuminated(False)

def test():
	fwd_rev(100)

def callback(cmd):
	l = nxt.Motor(b, nxt.PORT_A)
	r = nxt.Motor(b, nxt.PORT_B)
	if cmd.linear.x!=0.0:
		m = nxt.SynchronizedMotors(l, r, 0) #the last arg is turn ratio
		m.turn(90, (cmd.linear.x*10)) #forward  
		m.idle()
	elif cmd.angular.z>0.0:
		m = nxt.SynchronizedMotors(r, l, 127)
		m.turn(90, (cmd.angular.z*10))
		m.idle()
	elif cmd.angular.z<0.0:
		m = nxt.SynchronizedMotors(l, r, 127)
		m.turn(90, abs(cmd.angular.z*10))
		m.idle()

global direction
direction=True

def light_callback(msg):
#def light_callback():
	l = nxt.Motor(b, nxt.PORT_C)
	i=0
	intensity=0
	global direction
	if direction:
		try:
			while i<=8:
				l.turn(50, 20)
				time.sleep(.15)
				sample = light.get_sample()
				if sample>intensity:
					intensity=sample
				i=i+1
		except:
			print "Blocked"
			l.idle()
		direction=False
	else:
		try:
			while i<=8:
				l.turn(-50, 20)
				time.sleep(.15)
				sample = light.get_sample()
				if sample>intensity:
					intensity=sample
				i=i+1
		except:
			print "Blocked"
			l.idle()
		direction=True
	pub_intensity.publish(intensity)
#	print intensity
		
		

if __name__ == '__main__':
	print b.get_device_info();
	try:
		pub_intensity = rospy.Publisher('/nxt2/intensity', UInt16)
		rospy.Subscriber("/nxt2/cmd_vel", Twist, callback)
		rospy.Subscriber("/nxt2/find_intensity", Empty, light_callback)
		rospy.init_node('nxt2_pub_node', anonymous=True)
		r = rospy.Rate(10) # 20hz
		while not rospy.is_shutdown():
#			light_callback()		
			r.sleep()
	except rospy.ROSInterruptException: 
		pass
