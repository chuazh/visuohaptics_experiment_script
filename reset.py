#!/usr/bin/env python

import rospy
import dvrk
import numpy as np

#!/usr/bin/env python

import rospy
import dvrk
import numpy as np
import signal
import PyKDL
import load_test_4 as ld4
'''
""" MTM home position """
MTMR_cart = PyKDL.Vector(0.055288515671,-0.0508310176185,-0.0659661913251)
MTMR_rot = PyKDL.Rotation()
MTMR_rot = MTMR_rot.Quaternion(0.750403138242,-0.0111643539824,0.657383142871,-0.0679550644629)
MTMR_pos = PyKDL.Frame(MTMR_rot,MTMR_cart)

""" PSM home position """
PSM_cart = PyKDL.Vector(0.148006040795,-0.0687030860179,-0.0956039196032)
PSM_rot = PyKDL.Rotation()
PSM_rot = PSM_rot.Quaternion( 0.774206696711,-0.027368478383,0.629581117745,0.0590133318671)
PSM_pos = PyKDL.Frame(PSM_rot,PSM_cart)
'''
PSM_pos = ld4.load_manipulator_pose('./manipulator_homing/psm_home.txt')
MTMR_pos = ld4.load_manipulator_pose('./manipulator_homing/mtm_home.txt')
p2 = dvrk.psm('PSM2')
m2 = dvrk.mtm('MTMR')
c = dvrk.console()

print 'resetting arm...'

c.teleop_stop()
p2.move(PSM_pos)
m2.move(MTMR_pos)

print 'arm ready!'

c.teleop_start()
