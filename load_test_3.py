#!/usr/bin/env python

""" 
Script to perform tissue loading
by Zonghe Chua 08/04/18

This script will repeatedly move the right PSM in the vertical direction by a pre-specified amount that can be set with the target_displacement variable.

The home position can be set by specifying the cart1 and rot1 PyKDL variables. During homing the grippers will open, move and finally close (to grip the tissue sample) before performing the displacement.

This specific script doesn't cause the grippers to engage
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

sampling_period = 1/30
time_for_stretch = 6
target_displacement = 0.06

delta_displacement = -(target_displacement/time_for_stretch)*1/30

translation = PyKDL.Vector(delta_displacement,0.0,0.0)

translation2 = PyKDL.Vector(-delta_displacement,0.0,0.0)

"""define the waypoint positions for the PSMs for this load test"""
cart1 = PyKDL.Vector(0.1399,-0.0674,-0.0864)
rot1 = PyKDL.Rotation()
rot1 = rot1.Quaternion(0.732,0.013,0.677,0.0710)
pos1 = PyKDL.Frame(rot1,cart1)

cart2 = PyKDL.Vector(0.14514,-0.0666,-0.09294)
rot2 = PyKDL.Rotation()
rot2 = rot1.Quaternion(0.7209,-0.0192,0.6925,0.0152)
pos2 = PyKDL.Frame(rot2,cart2)

cart3 = PyKDL.Vector(0.14114,-0.0666,-0.09294)
rot3 = PyKDL.Rotation()
rot3 = rot1.Quaternion(0.7209,-0.0192,0.6925,0.0152)
pos3 = PyKDL.Frame(rot3,cart3)
p2 = dvrk.psm('PSM2')

# set our rate to 30hz
rate = rospy.Rate(30)

# get user to input filename
filename = raw_input("Please key in filename :")
filename = filename+'.csv'
print '\n'

print 'homing to position...'

#p2.open_jaw()
#p2.move(pos1)
p2.move(pos2)
p2.close_jaw();
rospy.sleep(1)

print 'performing stretch test...'
print 'displacement rate= ' + str(target_displacement/time_for_stretch) + ' m/s'

total_displacement = 0

""" initialize our data array """
time_start = rospy.get_time()
pose = p2.get_current_position()
twist = p2.get_current_twist_body()
wrench = p2.get_current_wrench_body()
#jaw_pos1 = p1.get_current_jaw_position()
#jaw_pos2 = p2.get_current_jaw_position()
#jaw_vel1 = p1.get_current_jaw_velocity()
#jaw_vel2 = p2.get_current_jaw_velocity()
#jaw_torq1 = p1.get_current_jaw_effort()
#jaw_torq2 = p2.get_current_jaw_effort()
time = rospy.get_time()-time_start

pos = get_cartesian(pose)
data = np.hstack((time,pos,twist,wrench))

# do a very discernable burst movement for timestamp purposes.
#p2.move(pos3)
#p2.move(pos2)

# stretch portion
while total_displacement < target_displacement :

	p2.dmove(translation)
	
	time = rospy.get_time()-time_start
	pose = p2.get_current_position()
	twist = p2.get_current_twist_body()
	wrench = p2.get_current_wrench_body()
	
	pos = get_cartesian(pose)
	new_data = np.hstack((time,pos,twist,wrench))
	data = np.vstack((data,new_data))
	
	total_displacement += -delta_displacement
	

total_displacement = 0 #reset our counter

# return portion
while total_displacement < target_displacement :

	p2.dmove(translation2)
	
	time = rospy.get_time()-time_start
	pose = p2.get_current_position()
	twist = p2.get_current_twist_body()
	wrench = p2.get_current_wrench_body()
	
	pos = get_cartesian(pose)
	
	new_data = np.hstack((time,pos,twist,wrench))
	data = np.vstack((data,new_data))
	
	total_displacement += -delta_displacement

#reset
#p2.open_jaw()
rospy.sleep(0.5)
#p2.move(pos1)

print 'test done'

print '\n'
print 'saving '+filename+'...'

np.savetxt(filename,data,delimiter=',',fmt='%.4f')































