#!/usr/bin/env python

""" 
Script to perform tissue loading in Palpation
by Zonghe Chua 11/18/19

This script will repeatedly move the right PSM in the vertical direction by a pre-specified amount that can be set with the target_displacement variable.

"""

import rospy
import dvrk
import numpy as np
import signal
import PyKDL
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3, Quaternion, Wrench, Pose
import numpy.matlib as npm
import copy


            
def haptic_feedback(data):
    global force_feedback
    force_feedback = [0, 0, 0]
    force_feedback[0] = data.force.z
    force_feedback[1] = -data.force.y
    force_feedback[2] = -data.force.x

def EP_pose(data):
    '''
    Input: data (ROS pose message)
    Output: ep_pose is a global variable that gets updated when the callback function is executed by the subscriber
    '''

    global ep_pose
    ep_pose = [0,0,0,0,0,0,0]
    ep_pose[0] = data.position.x
    ep_pose[1] = data.position.y
    ep_pose[2] = data.position.z
    ep_pose[3] = data.orientation.x
    ep_pose[4] = data.orientation.y
    ep_pose[5] = data.orientation.z
    ep_pose[6] = data.orientation.w

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
    ep_sub = rospy.Subscriber('/ep_pose', Pose, EP_pose)
    p2 = dvrk.psm('PSM2')

    PSM_pose = load_manipulator_pose('./manipulator_homing/psm_home_palp.txt')
    pos2 = get_cartesian(PSM_pose)

    """define the waypoint positions for the PSMs for this load test"""

    ref_force_array = np.array([8])

    # set our looprate
    loopRate = 150
    rate = rospy.Rate(loopRate)

    # get user to input filename
    filename = raw_input("Please key in filename :")
    filename = filename + '.csv'
    print '\n'

    """ initialize our data array """
    time_start = rospy.get_time()
    pose = p2.get_current_position()
    wrench = force_feedback
    ref_force = 0
    count = 0
    time = rospy.get_time() - time_start
    ep_pose = [0,0,0,0,0,0,0]
    pos = get_cartesian(pose)
    data = np.hstack((ref_force, count, time, pos, ep_pose, wrench))
    time_old = 0
    
    for ref_force in ref_force_array:

        for count in range(1, 3):

            print 'homing to position...'

            # zero-ing procedure
            p2.move(PSM_pose) 
            p2.close_jaw();
            rospy.sleep(2)

            time_start = rospy.get_time()
            
            target_force = ref_force # set the new target displacement to our ref displacement
            
            stretch_velocity = 0.001
            update_freq = loopRate
            delta_displacement = -stretch_velocity/update_freq

            translation = PyKDL.Vector(0.0, 0.0, delta_displacement)
            translation2 = PyKDL.Vector(0.0, 0.0, -delta_displacement)

            print 'performing stretch test ' + str(target_force) + ' iteration number ' + str(count)
            #print 'displacement rate= ' + str(target_displacement / time_for_stretch) + ' m/s'

            # move to the correct start loading position

            total_displacement = 0
            initial_position = ep_pose[1]/1000 #pose.p.x()

            # stretch portion
            #while total_displacement < target_displacement:
            while -force_feedback[0]<ref_force:
                p2.dmove(translation,interpolate = False)
                
                time = rospy.get_time() - time_start
                
                pose = p2.get_current_position()
                wrench = force_feedback

                pos = get_cartesian(pose)
                new_data = np.hstack((ref_force, count, time, pos, ep_pose,wrench))
                data = np.vstack((data, new_data))

                total_displacement =  ep_pose[1]/1000 - initial_position#pose.p.x()
                print('total displacement:' + str(total_displacement))
                print('force:' + str(force_feedback[0]))
                rate.sleep()

            print(total_displacement/time)
            print('returning...')
            print('total displacement : ' + str(total_displacement))
            print('force level : ' + str(force_feedback[0]))
            total_displacement = 0  # reset our counter
            initial_position =  ep_pose[1]/1000# pose.p.x()

            # return portion
            #while total_displacement < target_displacement:
            while -force_feedback[0]>0:
                p2.dmove(translation2,interpolate = False )

                time = rospy.get_time() - time_start
                pose = p2.get_current_position()
                twist = p2.get_current_twist_body()
                wrench = force_feedback

                pos = get_cartesian(pose)

                new_data = np.hstack((ref_force, count, time, pos, ep_pose, wrench))
                data = np.vstack((data, new_data))

                total_displacement = initial_position - ep_pose[1]/1000  #total_displacement = pose.p.x() - initial_position
                
                rate.sleep()
                
            rospy.sleep(0.5)

            print 'test done'
            print '\n'
            print 'saving ' + filename + '...'
            np.savetxt(filename, data, delimiter=',', fmt='%.4f')
