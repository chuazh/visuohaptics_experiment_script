#!/usr/bin/env python

import rospy
import dvrk
import numpy as np
import signal

def signal_handler(signal,frame):
	global interrupted
	interrupted = True

def get_cartesian(pose):
	position = pose.p	
	x = position.x()
	y = position.y()
	z = position.z()
	output = np.array([x,y,z])
	return output

signal.signal(signal.SIGINT,signal_handler)
interrupted = False

print "initializing logger"

filename = raw_input("Please key in filename :")
filename = filename+'.csv'

p1 = dvrk.psm('PSM1')
p2 = dvrk.psm('PSM2')

rate = rospy.Rate(30)

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

# initialize our np array
pos1 = get_cartesian(pose1)
pos2 = get_cartesian(pose2)

data = np.hstack((trial_num,time,pos1,pos2,twist1,twist2,wrench1,wrench2))

print "logging data..."

while trial_num<16:

	print ' trial '+str(trial_num)+' start...'
	time_start = rospy.get_time()
	

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
		print new_data
		data = np.vstack((data,new_data))
		
		if interrupted:
			trial_num += 1
			break	

		rate.sleep()	

	interrupted = False
	print ' trial '+str(trial_num-1)+' completed'

#scipy.io.savemat('test.mat',dat = data)
print '\n'
print 'saving '+filename+'...'

np.savetxt(filename,data,delimiter=',',fmt='%.4f')
