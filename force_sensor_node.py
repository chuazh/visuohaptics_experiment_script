#!/usr/bin/env python

import socket
import rospy
from geometry_msgs.msg import Vector3, Quaternion, Wrench, Pose

HOST = '192.168.1.5'
PORT = 27015

Fdata = Wrench()
Pdata = Pose()

rospy.init_node('force_sensor')
Fpub = rospy.Publisher('force_sensor', Wrench, queue_size=10)
Ppub = rospy.Publisher('ep_pose', Pose, queue_size=10)

rate = rospy.Rate(60)


print 'initializing ROS node...'

s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)

s.connect((HOST,PORT))

print 'connected!'

while True:
	fx = s.recv(7)
	fy = s.recv(7)
	fz = s.recv(7)
	px = s.recv(7)
	py = s.recv(7)
	pz = s.recv(7)
	q1 = s.recv(7)
	q2 = s.recv(7)
	q3 = s.recv(7)
	q4 = s.recv(7)

	Fdata.force.x = float(fx)
	Fdata.force.y = float(fy)
	Fdata.force.z = float(fz)
	Fdata.torque.x = 0
	Fdata.torque.y = 0
	Fdata.torque.z = 0

	Pdata.position.x = float(px)
	Pdata.position.y = float(py)
	Pdata.position.z = float(pz)
	Pdata.orientation.x = float(q1)
	Pdata.orientation.y = float(q2)
	Pdata.orientation.z = float(q3)
	Pdata.orientation.w = float(q4)

	Fpub.publish(Fdata)
	Ppub.publish(Pdata)
	rate.sleep()

s.close()

