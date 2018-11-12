#!/usr/bin/env python

import rospy
import dvrk
import numpy as np
import signal
import PyKDL
from sensor_msgs.msg import Joy

trigger = False # create a variable to store the trigger boolean
flag_next = False # create a flag variable for the trigger

# define the "home" positions for the manipulators of interest
init_cart = PyKDL.Vector(0.113,-0.089,-0.100)
init_rot = PyKDL.Rotation()
init_rot = init_rot.Quaternion(0.720,0.022,0.692,0.015)
init_pos = PyKDL.Frame(init_rot,init_cart)

MSM_cart = PyKDL.Vector(-0.128,-0.020,-0.214)
MSM_rot = PyKDL.Rotation()
MSM_rot = init_rot.Quaternion(0.779,-0.063,0.622,-0.017)
MSM_pos = PyKDL.Frame(MSM_rot,MSM_cart)

# utility functions

def trigger_callback(data):
	global trigger
	butt = data.buttons[0]	
	if butt > 0.5:
		trigger = True
	else:
		trigger = False	
	return

def get_cartesian(pose):
	position = pose.p	
	x = position.x()
	y = position.y()
	z = position.z()
	output = np.array([x,y,z])
	return output

print "initializing logger"

# get user to input filename
filename = raw_input("Please key in filename :")
filename = filename+'.csv'

# declare 2 objects
p1 = dvrk.psm('PSM1')
p2 = dvrk.psm('PSM2')
m2 = dvrk.mtm('MTMR')
c = dvrk.console()

# create the subscriber to check the footpedal
sub = rospy.Subscriber('/dvrk/footpedals/camera',Joy,trigger_callback)

# set our record rate to 30hz
rate = rospy.Rate(30)

#initialize our data array

time_start = rospy.get_time()
pose1 = p1.get_current_position()
pose2 = p2.get_current_position()
twist1 = p1.get_current_twist_body()
twist2 = p2.get_current_twist_body()
wrench1 = p1.get_current_wrench_body()
wrench2 = p2.get_current_wrench_body()
#jaw_pos1 = p1.get_current_jaw_position()
#jaw_pos2 = p2.get_current_jaw_position()
#jaw_vel1 = p1.get_current_jaw_velocity()
#jaw_vel2 = p2.get_current_jaw_velocity()
#jaw_torq1 = p1.get_current_jaw_effort()
#jaw_torq2 = p2.get_current_jaw_effort()
time = rospy.get_time()-time_start
trial_num = 1
pos1 = get_cartesian(pose1)
pos2 = get_cartesian(pose2)

data = np.hstack((trial_num,time,pos1,pos2,twist1,twist2,wrench1,wrench2))

c.teleop_start()

while trial_num<16:
	'''
	print ' trial '+str(trial_num)+' start...'

	print 'resetting arm...'
	c.teleop_stop()
	p2.move(init_pos)
	m2.move(MSM_pos)
	print 'arm ready!'
	c.teleop_start()
	'''
	
	
	time_start = rospy.get_time() # this re-initializes the start time for each trial
	next_flag = False #reset the flag


	while True:
		
		time = rospy.get_time()-time_start
		pose1 = p1.get_current_position()
		pose2 = p2.get_current_position()
		twist1 = p1.get_current_twist_body()
		twist2 = p2.get_current_twist_body()
		wrench1 = p1.get_current_wrench_body()
		wrench2 = p2.get_current_wrench_body()
		#jaw_pos1 = p1.get_current_jaw_position()
		#jaw_pos2 = p2.get_current_jaw_position()
		#jaw_vel1 = p1.get_current_jaw_velocity()
		#jaw_vel2 = p2.get_current_jaw_velocity()
		#jaw_torq1 = p1.get_current_jaw_effort()
		#jaw_torq2 = p2.get_current_jaw_effort()
		pos1 = get_cartesian(pose1)
		pos2 = get_cartesian(pose2)
		new_data = np.hstack((trial_num,time,pos1,pos2,twist1,twist2,wrench1,wrench2))
		data = np.vstack((data,new_data))

		if next_flag == False and trigger == True and time>1:
			next_flag = True
		
		if next_flag == True: 
			trial_num += 1			
			break

		rate.sleep()	


	print ' trial '+str(trial_num-1)+' completed'

#scipy.io.savemat('test.mat',dat = data)
print '\n'
print 'saving '+filename+'...'

np.savetxt(filename,data,delimiter=',',fmt='%.4f')
