#!/usr/bin/env python

""" 
Script to perform tissue loading
by Zonghe Chua 08/04/18

This script will repeatedly move the right PSM in the vertical direction by a pre-specified amount that can be set with the target_displacement variable.

The home position can be set by specifying the cart1 and rot1 PyKDL variables. During homing the grippers will open, move and finally close (to grip the tissue sample) before performing the displacement.
"""


import rospy
import dvrk
import numpy as np
import signal
import PyKDL
from sensor_msgs.msg import Joy

def get_cartesian(pose):
	position = pose.p	
	x = position.x()
	y = position.y()
	z = position.z()
	output = np.array([x,y,z])
	return output

"""define the waypoint positions for the PSMs for this load test"""
cart1 = PyKDL.Vector(0.14514,-0.0666,-0.09294)
rot1 = PyKDL.Rotation()
rot1 = rot1.Quaternion(0.7209,-0.0192,0.6925,0.0152)
pos1 = PyKDL.Frame(rot1,cart1)

p2 = dvrk.psm('PSM2')

# set our rate to 30hz
rate = rospy.Rate(30)

print 'homing to position...'
p2.move(pos1)

































