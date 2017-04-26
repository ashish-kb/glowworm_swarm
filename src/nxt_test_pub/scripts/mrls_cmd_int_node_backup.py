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
import time
import serial

global serial_port

def callback0(cmd):
	if cmd.linear.x!=0.0:
		print "linear"
		serial_port.write("at"+chr(int(cmd.linear.x*255)));
	elif cmd.angular.z<0.0:
		print "angular left"
		serial_port.write("ar"+chr(128+(int(cmd.angular.z*126))));
	elif cmd.angular.z>0.0:
		print "angular right"
		serial_port.write("ar"+chr(128+(int(cmd.angular.z*126))));


def light_callback0(msg):
	serial_port.write("air");
	intensity=serial_port.readline()
	print intensity
	
def callback(cmd):
	if cmd.linear.x!=0.0:
		print "linear"
		serial_port.write("at"+chr(cmd.linear.x*255));
	elif cmd.angular.z>0.0:
		print "angular left"
		serial_port.write("ar"+chr(128+(cmd.angular.z*127)));
	elif cmd.angular.z<0.0:
		print "angular right"
		serial_port.write("ar"+chr(cmd.angular.z*128));


def light_callback(msg):
	serial_port.write("ar"+chr(cmd.liner.x*128));
	intensity=serial_port.readline()
	print intensity
	

if __name__ == '__main__':
	serial_port = serial.Serial('/dev/ttyUSB1', 115200)
	
	try:
		pub_intensity = rospy.Publisher('/mrlsb0/intensity', UInt16)
		pub_intensity = rospy.Publisher('/mrlsb1/intensity', UInt16)
		pub_intensity = rospy.Publisher('/mrlsb2/intensity', UInt16)
		pub_intensity = rospy.Publisher('/mrlsb3/intensity', UInt16)
		pub_intensity = rospy.Publisher('/mrlsb4/intensity', UInt16)

		rospy.Subscriber("/mrlsb0/cmd_vel", Twist, callback0)
		rospy.Subscriber("/mrlsb0/find_intensity", Empty, light_callback0)
		
		rospy.Subscriber("/mrlsb1/cmd_vel", Twist, callback)
		rospy.Subscriber("/mrlsb1/find_intensity", Empty, light_callback)
		
		rospy.Subscriber("/mrlsb2/cmd_vel", Twist, callback)
		rospy.Subscriber("/mrlsb2/find_intensity", Empty, light_callback)	
		
		rospy.Subscriber("/mrlsb3/cmd_vel", Twist, callback)
		rospy.Subscriber("/mrlsb3/find_intensity", Empty, light_callback)		
		
		rospy.Subscriber("/mrlsb4/cmd_vel", Twist, callback)
		rospy.Subscriber("/mrlsb4/find_intensity", Empty, light_callback)		
		
		
		rospy.init_node('mrl_cmd_int_node', anonymous=True)
		r = rospy.Rate(10) # 20hz
		while not rospy.is_shutdown():	
#			light_callback()			
			r.sleep()
	except rospy.ROSInterruptException: 
		pass
