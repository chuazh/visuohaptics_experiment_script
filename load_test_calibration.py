#!/usr/bin/env python

import rospy
import dvrk
import numpy as np
import signal
import PyKDL
import curses
from sensor_msgs.msg import Joy

""" 
Script to perform incremental tissue loading
by Zonghe Chua 08/04/18

This script will allow the user to incrementally move the PSM gripper by 1mm when they press the 'w' character on the keyboard. To print the displacement press 'd' and to exit the program press 'q'

The home position can be set by specifying the cart1 and rot1 PyKDL variables. During homing the grippers will open, move and finally close (to grip the tissue sample) before performing the displacement.
"""

def main(screen):
	
	screen.nodelay(1)
		
	# this sets the increment for our displacement
	delta = 0.001
	
	total_translation = 0.0
	
	translation = PyKDL.Vector(0.0,0.0,delta)
	
	'''define the waypoint positions for the PSMs for this load test'''
	cart1 = PyKDL.Vector(0.116,-0.090,-0.083)
	rot1 = PyKDL.Rotation()
	rot1 = rot1.Quaternion(0.694,0.0113,0.719,-0.005)
	pos1 = PyKDL.Frame(rot1,cart1)
	

	p2 = dvrk.psm('PSM2')
	c = dvrk.console()
	
	# set our rate to 30hz
	rate = rospy.Rate(30)

	print 'homing to position...'
	c.teleop_stop()
	#p2.open_jaw()
	#p2.move(pos1)
	p2.close_jaw()
	
	while True:
		
		if screen.getch() == ord('w'):
			p2.dmove(translation)
			total_translation = total_translation + delta		

		if screen.getch() == ord('d'):
			print 'displacement is: '
			print total_translation
		
		if screen.getch() == ord('q'):
			print 'quitting'
			break
			
	c.teleop_start()

	
if __name__ == "__main__":
	
	curses.wrapper(main)
	



