#!/usr/bin/env python

import socket
import rospy
from geometry_msgs.msg import Vector3, Quaternion, Wrench

HOST = '192.168.1.5'
PORT = 27015

data = Wrench()

rospy.init_node('force_sensor')
pub = rospy.Publisher('force_sensor', Wrench, queue_size=10)
rate = rospy.Rate(60)


print 'initializing ROS node...'

s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)

s.connect((HOST,PORT))

print 'connected!'

while True:
	x = s.recv(7)
	y = s.recv(7)
	z = s.recv(7)
	
	data.force.x = float(x)
	data.force.y = float(y)
	data.force.z = float(z)
	data.torque.x = 0
	data.torque.y = 0 
	data.torque.z = 0

	pub.publish(data)
	rate.sleep()

s.close()

