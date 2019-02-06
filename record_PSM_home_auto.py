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

def haptic_feedback(data):
    global force_feedback
    force_feedback = [0, 0, 0]
    force_feedback[0] = data.force.x
    force_feedback[1] = data.force.y
    force_feedback[2] = -data.force.z

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

def zero_forces(PSM,epsilon):
    home = False
    Kp = 0.007
    Kd = 0.005

    F_old = force_feedback

    while home == False:

        Fx = force_feedback[0]
        Fy = force_feedback[1]
        Fz = force_feedback[2]
        Fx_d = Fx-F_old[0]
        Fy_d = Fy-F_old[1]
        Fz_d = Fz-F_old[2]
        F_old = [Fx,Fy,Fz]

        if np.linalg.norm(force_feedback)>epsilon:
            PSM.dmove(PyKDL.Vector(Kp*Fx+Kd*Fx_d, Kp*Fz+Kd*Fz_d, -(Kp*Fy+Kd*Fy_d)))
            print(np.linalg.norm(force_feedback))
            #print(str(Kp*Fx+Kd*Fx_d) + ',' +str(Kp*Fy+Kd*Fy_d) + ',' +str(-(Kp*Fz+Kd*Fz_d)))
            print(str(Kp*Fx) + ',' +str(Kp*Fy) + ',' +str(-(Kp*Fz)))
        else:
            home = True

if __name__ == "__main__":

    """ MTM home rough position """
    ''' We use this to initialize a position for the MTMR'''
    MTMR_pos = load_manipulator_pose('./manipulator_homing/mtm_home.txt')

    c = dvrk.console()
    p2 = dvrk.psm('PSM2')
    mtm = dvrk.mtm('MTMR')
    sub = rospy.Subscriber('/dvrk/footpedals/camera', Joy, trigger_callback)
    force_sub = rospy.Subscriber('/force_sensor', Wrench, haptic_feedback)

    force_feedback = [0,0,0]

    print('initializing approximate MTMR position')
    mtm.move(MTMR_pos)
    c.teleop_start()

    f = input('Enter 1 to do auto homing, 2 to do manual homing... : ')

    filename = './manipulator_homing/'
    if f == 1:
        zero_forces(p2,0.01)

        frame_psm = p2.get_current_position()
        frame_mtm = mtm.get_current_position()

        output_psm = get_pose(frame_psm)
        output_mtm = get_pose(frame_mtm)

        print("====== data recorded ======")
        print("PSM Pose")
        print("x= " + str(output_psm[0]))
        print("y= " + str(output_psm[1]))
        print("z= " + str(output_psm[2]))
        print("q1= " + str(output_psm[3]))
        print("q2= " + str(output_psm[4]))
        print("q3= " + str(output_psm[5]))
        print("q4= " + str(output_psm[6]))
        save_file = filename+'psm_home.txt'
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
        save_file = filename+'mtm_home.txt'
        print("saving to " + save_file + " ...")
        np.savetxt(save_file,output_mtm,delimiter=',')
        print("file saved")

    else:

        trigger = False
        trigger_time = 0

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
                save_file = filename+'psm_home.txt'
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
                save_file = filename+'mtm_home.txt'
                print("saving to " + save_file + " ...")
                np.savetxt(save_file,output_mtm,delimiter=',')
                print("file saved")

                print("continue recording if desired...")
                print("===========================")
