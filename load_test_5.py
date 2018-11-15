#!/usr/bin/env python

""" 
Script to perform tissue loading
by Zonghe Chua 08/04/18

This script will repeatedly move the right PSM in the vertical direction by a pre-specified amount that can be set with the target_displacement variable.

The home position can be set by specifying the cart1 and rot1 PyKDL variables. During homing the grippers will open, move and finally close (to grip the tissue sample) before performing the displacement.

This specific script test loading at a displacement other than the neutral
"""

import rospy
import dvrk
import numpy as np
import signal
import PyKDL
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3, Quaternion, Wrench

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
            #print(np.linalg.norm(force_feedback))
            #print(str(Kp*Fx+Kd*Fx_d) + ',' +str(Kp*Fy+Kd*Fy_d) + ',' +str(-(Kp*Fz+Kd*Fz_d)))
        else:
            home = True

def haptic_feedback(data):
    global force_feedback
    force_feedback = [0, 0, 0]
    force_feedback[0] = data.force.x
    force_feedback[1] = data.force.y
    force_feedback[2] = -data.force.z


def get_cartesian(pose):
    position = pose.p
    x = position.x()
    y = position.y()
    z = position.z()
    output = np.array([x, y, z])
    return output


def load_manipulator_pose(filename):
    data = np.loadtxt(filename, delimiter=',')
    Rot = PyKDL.Rotation()
    Rot = Rot.Quaternion(data[3], data[4], data[5], data[6])
    Pos = PyKDL.Vector(data[0], data[1], data[2])
    Frame = PyKDL.Frame(Rot, Pos)

    return Frame

if __name__ == "__main__":

    ''' Initialize variables and environment'''

    sampling_period = 1 / 100
    force_feedback = [0, 0, 0]

    force_sub = rospy.Subscriber('/force_sensor', Wrench, haptic_feedback)
    p2 = dvrk.psm('PSM2')
    zero_forces(p2,0.05)

    PSM_pose = load_manipulator_pose('./manipulator_homing/psm_home.txt')
    pos2 = get_cartesian(PSM_pose)

    """define the waypoint positions for the PSMs for this load test"""

    # ref_displacement_array = np.array([0.002786,0.00615,0.008032,0.01015,0.01488,0.019,0.02104,0.02621,0.02885,0.03334])
    # ref_displacement_array = np.array([0.128903083704,0.107929114901,0.0934401071415,0.085339802604,0.0806550459682,0.0772358149271])
    #ref_displacement_array = np.array([0.01,0.02,0.03,0.04,0.045])
    #ref_displacement_array = np.array([0.008,0.025,0.031,0.035,0.042])
    ref_displacement_array = np.array([0.025,0.031,0.035,0.042])
    #ref_displacement_array = np.array([0.03])
    # set our rate to 1000hz
    rate = rospy.Rate(1000)

    # get user to input filename
    filename = raw_input("Please key in filename :")
    filename = filename + '.csv'
    print '\n'

    """ initialize our data array """
    time_start = rospy.get_time()
    pose = p2.get_current_position()
    wrench = force_feedback
    ref_displacement = 0
    count = 0
    time = rospy.get_time() - time_start

    pos = get_cartesian(pose)
    data = np.hstack((ref_displacement, count, time, pos, wrench))
    
    for y in range(1,15):
    
        for ref_displacement in ref_displacement_array:

            for count in range(1, 2):

                print 'homing to position...'

                p2.move(PSM_pose)
                p2.close_jaw();
                rospy.sleep(2)
                zero_forces(p2,0.05)

                time_start = rospy.get_time()

                x_0 = p2.get_current_position()
                print(x_0.p.x())
                #target_displacement =  x_0.p.x() - ref_displacement
                target_displacement = ref_displacement
                time_for_stretch = target_displacement * 100
                delta_displacement = -(target_displacement / time_for_stretch) * 1 / 30
                translation = PyKDL.Vector(delta_displacement, 0.0, 0.0)
                translation2 = PyKDL.Vector(-delta_displacement, 0.0, 0.0)

                print 'performing stretch test ' + str(target_displacement) + ' iteration number ' + str(count)
                print 'displacement rate= ' + str(target_displacement / time_for_stretch) + ' m/s'

                # move to the correct start loading position

                total_displacement = 0
                initial_position = pose.p.x()

                # stretch portion
                while total_displacement < target_displacement:
                    p2.dmove(translation)

                    time = rospy.get_time() - time_start
                    pose = p2.get_current_position()
                    wrench = force_feedback

                    pos = get_cartesian(pose)
                    new_data = np.hstack((ref_displacement, count, time, pos, wrench))
                    data = np.vstack((data, new_data))

                    total_displacement = initial_position - pose.p.x()


                print('returning...')
                print('total displacement : ' + str(total_displacement))
                print('force level : ' + str(force_feedback[0]))
                total_displacement = 0  # reset our counter
                initial_position = pose.p.x()

                # return portion
                while total_displacement < target_displacement:
                    p2.dmove(translation2)

                    time = rospy.get_time() - time_start
                    pose = p2.get_current_position()
                    twist = p2.get_current_twist_body()
                    wrench = force_feedback

                    pos = get_cartesian(pose)

                    new_data = np.hstack((ref_displacement, count, time, pos, wrench))
                    data = np.vstack((data, new_data))

                    total_displacement = pose.p.x() - initial_position

                rospy.sleep(0.5)

                print 'test done'
                print '\n'
                print 'saving ' + filename + '...'
                np.savetxt(filename, data, delimiter=',', fmt='%.4f')
