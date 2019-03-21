#!/usr/bin/env python

import rospy
import dvrk
import numpy as np
import signal
import PyKDL
import os
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3, Quaternion, Wrench
from std_msgs.msg import String, Bool

def trigger_callback(data):
    global trigger
    butt = data.buttons[0]
    if butt > 0.5:
        trigger = True
    else:
        trigger = False
    return

def get_pose(frame):

    x = frame.p.x()
    y = frame.p.y()
    z = frame.p.z()
    q1,q2,q3,q4 = frame.M.GetQuaternion()
    output = (x,y,z,q1,q2,q3,q4)

    return output

def load_manipulator_pose(filename):
    data = np.loadtxt(filename, delimiter=',')
    Rot = PyKDL.Rotation()
    Rot = Rot.Quaternion(data[3], data[4], data[5], data[6])
    Pos = PyKDL.Vector(data[0], data[1], data[2])
    Frame = PyKDL.Frame(Rot, Pos)

    return Frame

""" MTM home rough position """
''' We use this to initialize a position for the MTMR'''

'''MTMR_cart = PyKDL.Vector(-0.108202971153, -0.0521530973454, -0.0688619252091)
MTMR_rot = PyKDL.Rotation()
MTMR_rot = MTMR_rot.Quaternion(0.874974940577, 0.00410262693351, 0.483952213296, -0.0138664003756)'''
MTMR_pos = load_manipulator_pose('./manipulator_homing/mtm_home_palp.txt')

c = dvrk.console()
p2 = dvrk.psm('PSM2')
mtm = dvrk.mtm('MTMR')

print('initializing approximate MTMR position')
mtm.move(MTMR_pos)
c.teleop_start()

sub = rospy.Subscriber('/dvrk/footpedals/camera', Joy, trigger_callback)
trigger = False
trigger_time = 0
filename = './manipulator_homing/'
print("initialized recording")
print("step on GO pedal to record PSM and MTM position")
rate = rospy.Rate(1000)
time_init = rospy.get_time()

while not rospy.is_shutdown():
    time = rospy.get_time()-time_init

    frame_psm = p2.get_current_position()
    frame_mtm = mtm.get_current_position()

    output_psm = get_pose(frame_psm)
    output_mtm = get_pose(frame_mtm)

    if (time-trigger_time)>0.5 and trigger == True:
        trigger_time = time

        print("====== data recorded ======")
        print("PSM Pose")
        print("x= " + str(output_psm[0]))
        print("y= " + str(output_psm[1]))
        print("z= " + str(output_psm[2]))
        print("q1= " + str(output_psm[3]))
        print("q2= " + str(output_psm[4]))
        print("q3= " + str(output_psm[5]))
        print("q4= " + str(output_psm[6]))
        save_file = filename+'psm_home_palp.txt'
        print("saving to " + save_file + " ...")
        np.savetxt(save_file,output_psm,delimiter=',')
        print("file saved")

        print("MTM Pose")
        print("x= " + str(output_mtm[0]))
        print("y= " + str(output_mtm[1]))
        print("z= " + str(output_mtm[2]))
        print("q1= " + str(output_mtm[3]))
        print("q2= " + str(output_mtm[4]))
        print("q3= " + str(output_mtm[5]))
        print("q4= " + str(output_mtm[6]))
        save_file = filename+'mtm_home_palp.txt'
        print("saving to " + save_file + " ...")
        np.savetxt(save_file,output_mtm,delimiter=',')
        print("file saved")

        print("continue recording if desired...")
        print("===========================")

    rate.sleep()

