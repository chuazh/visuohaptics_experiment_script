#!/usr/bin/env python

""" 
Script to Pilot Test
by Zonghe Chua 09/03/18

This script run the pilot test study.

It requires initializing the MSM and PSM "home" positions.

"""

import rospy
import dvrk
import numpy as np
import signal
import PyKDL
import random
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3, Quaternion, Wrench

def get_cartesian(pose):
	position = pose.p	
	x = position.x()
	y = position.y()
	z = position.z()
	output = np.array([x,y,z])
	return output
	
def trigger_callback(data):
	global trigger
	butt = data.buttons[0]	
	if butt > 0.5:
		trigger = True
	else:
		trigger = False	
	return
	
def haptic_feedback(data):
	global force_feedback
	force_feedback = [0,0,0]
	force_feedback[0] = data.force.z
	force_feedback[1] = data.force.x
	force_feedback[2] = -data.force.y

sampling_period = 1/30


""" MTM home position """
MTMR_cart = PyKDL.Vector(-0.05824,-0.0546,-0.0533)
MTMR_rot = PyKDL.Rotation()
MTMR_rot = MTMR_rot.Quaternion(0.7232,0.0306,0.6875,0.05687)
MTMR_pos = PyKDL.Frame(MTMR_rot,MTMR_cart)

""" PSM home position """
PSM_cart = PyKDL.Vector(0.1429,-0.06655,-0.09587)
PSM_rot = PyKDL.Rotation()
PSM_rot = PSM_rot.Quaternion(0.652,-0.113558,0.7487,0.01616)
PSM_pos = PyKDL.Frame(PSM_rot,PSM_cart)

# Declare a list that holds our reference forces
training_forces = [1,1.5,2,2.75,4]
test_forces_ds = np.array([1,1,1,1.5,1.5,1.5,2,2,2,2.75,2.75,2.75,4,4,4])



# declare our PSM and MTMs
p2 = dvrk.psm('PSM2')
m2 = dvrk.mtm('MTMR')
c = dvrk.console()

# set our rate to 30hz
rate = rospy.Rate(30)

# create the subscriber to check the footpedal
sub = rospy.Subscriber('/dvrk/footpedals/camera',Joy,trigger_callback)
force_sub = rospy.Subscriber('/force_sensor',Wrench,haptic_feedback)

# initialize a flag to denote end of test trials
trigger = False # create a variable to store the trigger boolean
flag_next = False # create a flag variable for the trigger
exiter = False

filename = raw_input("Please key in subject name: ")
print '\n'

# get user to input filename
condition = input("Please enter 0 for training and 1 for test: ")
print '\n'

if condition ==0 :
	filename = filename+'_train'
else:
	filename = filename+'_test'



material_select = input("Please select material. 0 for EF and 1 for DS: ")

if material_select==0:
	filename = filename+'_ef'
else:
	filename = filename+'_ds'
	
c.teleop_start()
print("Please engage the gripper onto the tissue sample")

keypress = 0

while keypress==0:
	keypress=raw_input("Enter 1 to continue: ")


	
if condition == 0:

	print("starting training session")
	
	trial_num = 0
	prev_force = 1
	force_feedback = [0,0,0]
	
	while exiter == False:
		
		next_flag = False #reset the flag
		
		#home the MTMs and PSMs
		c.teleop_stop()
		print("homing MTM and PSM")
		p2.close_jaw()
		p2.move(PSM_pos)
		m2.move(MTMR_pos)
		rospy.sleep(2)
		#initialize our variables
		pose = p2.get_current_position()
		twist = p2.get_current_twist_body()
		wrench = p2.get_current_wrench_body()
		time_start = rospy.get_time() # this re-initializes the start time for each trial
		time = rospy.get_time()-time_start
		pos = get_cartesian(pose)
		
		ref_force = input("what is the force level for this trial? Please start force sensor data log before hitting enter: ")
		data = np.hstack((trial_num,ref_force,time,pos,twist,wrench))
		
		if ref_force == 17:
			exiter = True
			print("exiting...")
			break
		
		if prev_force-ref_force==0: 
			trial_num += 1
		else:
			trial_num = 1
					
		
		c.teleop_start()
		print('recording...trial ' +str(trial_num)+', ref force ' + str(ref_force))
		
		while True:
			
			m2.set_wrench_spatial_force(force_feedback)
			
			time = rospy.get_time()-time_start
			pose = p2.get_current_position()
			twist = p2.get_current_twist_body()
			wrench = p2.get_current_wrench_body()
	
			pos = get_cartesian(pose)
			new_data = np.hstack((trial_num,ref_force,time,pos,twist,wrench))
			data = np.vstack((data,new_data))
			
			if next_flag == False and trigger == True and time>1:
				next_flag = True

			if next_flag == True:
				
				print ('trial '+str(trial_num)+' completed')
				save_filename = filename + '_' + str(ref_force) + '_' + str(trial_num) + '.csv'
				print 'saving '+ save_filename+'...'
				np.savetxt(save_filename,data,delimiter=',',fmt='%.4f')
				
				break
		
		prev_force = ref_force


			
else:
	print("starting evaluation sessions")
	
	trial_num = 1
	
	#do a random shuffle of the array
	test_forces_ds = np.random.permutation(test_forces_ds)
	
	print test_forces_ds
	
	while exiter == False:
		
		next_flag = False #reset the flag
		
		#home the MTMs and PSMs
		c.teleop_stop()
		print("homing MTM and PSM")
		p2.close_jaw()
		p2.move(PSM_pos)
		m2.move(MTMR_pos)
		rospy.sleep(2)
		#initialize our variables
		pose = p2.get_current_position()
		twist = p2.get_current_twist_body()
		wrench = p2.get_current_wrench_body()
		time_start = rospy.get_time() # this re-initializes the start time for each trial
		time = rospy.get_time()-time_start
		pos = get_cartesian(pose)
		
		ref_force = test_forces_ds[trial_num-1]
		
		print('ref force is: ' + str(ref_force))
		
		data = np.hstack((trial_num,ref_force,time,pos,twist,wrench))
		
		c.teleop_start()
		print('recording...trial ' +str(trial_num)+', ref force ' + str(ref_force))
		
		while True:
			
			time = rospy.get_time()-time_start
			pose = p2.get_current_position()
			twist = p2.get_current_twist_body()
			wrench = p2.get_current_wrench_body()
	
			pos = get_cartesian(pose)
			new_data = np.hstack((trial_num,ref_force,time,pos,twist,wrench))
			data = np.vstack((data,new_data))
			
			if next_flag == False and trigger == True and time>1:
				next_flag = True
			
			if next_flag == True:
				
				save_filename = filename + '_' + str(trial_num) + '.csv'
				print 'saving '+save_filename+'...'
				np.savetxt(save_filename,data,delimiter=',',fmt='%.4f')
				
				trial_num += 1									
				print ('trial '+str(trial_num-1)+' completed')			
				break	

		if trial_num == 16:
			exiter = True
			print("exiting...")
	

