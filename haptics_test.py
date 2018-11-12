#!/usr/bin/env python



import rospy
import dvrk
import numpy as np
import signal
import PyKDL
from geometry_msgs.msg import Vector3, Quaternion, Wrench
from sensor_msgs.msg import Joy

#rospy.init_node('haptic_feedback')

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
	force_feedback[0] = data.force.x
	force_feedback[1] = data.force.y
	force_feedback[2] = -data.force.z

translation = PyKDL.Vector(0.0,0.0,0.0) #dummy to get the robot back in position control mode
force_feedback = [0,0,0]
m2 = dvrk.mtm('MTMR')

teleop_sub = rospy.Subscriber('/dvrk/footpedals/coag',Joy,trigger_callback)
force_sub = rospy.Subscriber('force_sensor',Wrench,haptic_feedback)

force_feedback = [0,0,0]
trigger = False
position_state_flag = True
m2.set_wrench_body_orientation_absolute(True)

while True:
	
	if trigger == True:
		#print force_feedback
		m2.set_wrench_body_force(force_feedback)
		
		if position_state_flag == True:
			#print m2.get_robot_state()
			position_state_flag = False
			
		
	else:
		if position_state_flag == False:
			m2.dmove(translation)
			#print m2.get_robot_state()
			position_state_flag = True
		