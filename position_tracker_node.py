#!/usr/bin/env python

import socket
import rospy
from geometry_msgs.msg import Vector3, Quaternion, Wrench, Pose
import os

HOST = '192.168.1.5'
PORT = 27015

Fdata = Wrench()
Pdata = Pose()

rospy.init_node('position_tracker_node')
Ppub = rospy.Publisher('ep_pose', Pose, queue_size=10)

rate = rospy.Rate(1000)


print 'initializing ROS node...'

s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)

s.connect((HOST,PORT))

print 'connected!'

old_time = 0

while True:
	time = rospy.get_time()
	fs = 1/(time-old_time)

	px = s.recv(5)
	py = s.recv(5)
	pz = s.recv(5)
	q1 = s.recv(5)
	q2 = s.recv(5)
	q3 = s.recv(5)
	q4 = s.recv(5)

	Pdata.position.x = float(px)
	Pdata.position.y = float(py)
	Pdata.position.z = float(pz)
	Pdata.orientation.x = float(q1)
	Pdata.orientation.y = float(q2)
	Pdata.orientation.z = float(q3)
	Pdata.orientation.w = float(q4)

	Ppub.publish(Pdata)
	rate.sleep()
	old_time = time

s.close()

