#!/usr/bin/env python

import socket
import rospy
from geometry_msgs.msg import Vector3, Quaternion, Wrench, Pose
import os
import struct

HOST = '192.168.1.5'
PORT = 27015

Fdata = Wrench()
Pdata = Pose()

rospy.init_node('position_tracker_node')
Ppub = rospy.Publisher('ep_pose', Pose, queue_size=10)

rate = rospy.Rate(90)

num_params = 7 # 3 position 4 orientation
size_float = 4 # a float is 4 bytes
total_pack_size = num_params*size_float

print 'initializing ROS node...'

s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
s.connect((HOST,PORT))

print 'connected!'

while True:
	
	# receive from the server 
	pack = s.recv(total_pack_size,socket.MSG_WAITALL)
	# unpack
	px,py,pz,q1,q2,q3,q4 = struct.unpack('<fffffff',pack)
	
	

	Pdata.position.x = float(px)
	Pdata.position.y = float(py)
	Pdata.position.z = float(pz)
	#Pdata.orientation.x = 0
	#Pdata.orientation.y = 0
	#Pdata.orientation.z = 0
	#Pdata.orientation.w = 0
	Pdata.orientation.x = float(q1)
	Pdata.orientation.y = float(q2)
	Pdata.orientation.z = float(q3)
	Pdata.orientation.w = float(q4)
	
	Ppub.publish(Pdata)
	rate.sleep()


s.close()

