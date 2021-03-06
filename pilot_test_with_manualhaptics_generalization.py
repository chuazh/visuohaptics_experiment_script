#!/usr/bin/env python

""" 
Script to Pilot Test With Manual Haptics and Generalization
by Zonghe Chua 03/15/19

This script run the pilot test study with manual haptics.
It also has a condition for testing with a rotated sample.
Future scripts might include a palpation test too.

It requires initializing the MSM and PSM "home" positions.

Log
03/18/19 Added the option to perform catch trials
"""

import rospy
import dvrk
import numpy as np
import signal
import PyKDL
import os
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3, Quaternion, Wrench, Pose
from std_msgs.msg import String, Bool
import numpy.matlib as npm
from random import randint

def trigger_callback(data):
    
    #Callback function for the utility footpedal that helps advance the experiment progression
    #Input : data (ROS joystick message)
    #Output : no real output, but trigger is a global boolean variable that toggles

    global trigger
    butt = data.buttons[0]
    if butt > 0.5:
        trigger = True
    else:
        trigger = False
    return

def trigger_callback2(data):
    '''
    The callback function for a button
    Input : data (ROS Bool message)
    Output: teleop is a global boolean variable that toggles
    '''
    
    global trigger
    butt = data.data
    if butt == True:
        trigger = True
        
    else:
        trigger = False
    return

def teleop_callback(data):
    '''
    The callback function for the teleoperation pedal
    Input : data (ROS joystick message)
    Output: teleop is a global boolean variable that toggles
    '''

    global teleop
    butt = data.buttons[0]
    if butt > 0.5:
        teleop = True
    else:
        teleop = False


def haptic_feedback(data):
    '''
    Input: data (ROS force message)
    Output: force_feedback is a global variable that gets updated when the callback function is executed by the subscriber
    '''

    global force_feedback
    force_feedback = [0, 0, 0]
    '''
    force_feedback[0] = data.force.x
    force_feedback[1] = data.force.y
    force_feedback[2] = -data.force.z
    '''
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


def collect_filename():
    '''
    This function collects the relevant parameters for our experiment
    It returns a numpy array of the subject number, the condition being tested, the training or test indicator, and the material
    '''

    subj = input("Please key in subject number: ")
    print '\n'

    haptic = input("Please enter 0 for no haptic condition, 1 for haptics, 2 for manual haptics: ")
    print '\n'

    test = input("Please enter 0 for training, 1 for test, 2 for rotation test, 3 for catch, 4 for palpate: ")
    print '\n'

    material_select = input("Please select material. 0 for EF and 1 for DS: ")

    return np.array([subj, haptic, test, material_select])


def populate_training(force_array, num_trials):
    '''
    This function populates a training sequence of forces based on the forces specified in the force_array with a multiple of num_trials for each
    Input : force_array (1xN list) , num_trials (positive integer)
    Output: force_array_seq (1x(N*num_trials) list)
    '''

    force_array_seq = np.zeros(num_trials * len(force_array))
    idx = 0
    for i in range(num_trials * len(force_array)):
        force_array_seq[i] = force_array[idx]
        if (i + 1) % num_trials == 0:
            idx += 1

    return force_array_seq

def populate_and_randomize_test_catch(force_array, num_trials,num_catch, catch_limit):
    '''
    This function populates a test sequence of forces based on the forces specified in force_array with multiples of num_trials.
    Alongside the test sequence, it also generates a flag to indicate catch trials. The number of catch trials is specified by num_catch
    The function shuffles the sequence of forces and makes sure that no test forces is repeated back to back. It also checks and reshuffles if
    the number of consecutive catch trials exceed the variable catch_limit.
    
    Input: force_array (1xN list) , num_trials (integer)
    Output: force_array_seq (1x(N*num_trials) list)
    '''

    force_array_seq = [[0,0] for i in range(num_trials * len(force_array))]

    idx = 0
    for i in range(num_trials * len(force_array)): # populate the sequence list with the forces
        force_array_seq[i][0] = force_array[idx]
        if (i + 1) % num_trials == 0:
            idx += 1

    count_catch = 0

    for i in range(num_trials * len(force_array)):

        if i % num_trials == 0:
            count_catch = 0

        if count_catch < num_catch:
            force_array_seq[i][1] = 1
        else:
            force_array_seq[i][1] = 0

        count_catch = count_catch + 1

    reset = True # this switch defines if we still have to shuffle
    catch_limiter = 0

    while reset == True:
        reset = False
        np.random.shuffle(force_array_seq)
        checkedNum = force_array_seq[0][0]
        checkedCatch = force_array_seq[0][1]

        force_array_tup = [[],[]]
        for i in range(0, num_trials * len(force_array)):
            force_array_tup[0].append(force_array_seq[i][0])
            force_array_tup[1].append(force_array_seq[i][1])

        #print(force_array_tup[0])
        #print(force_array_tup[1])

        for i in range(1, num_trials * len(force_array)):
            test = (checkedNum == force_array_seq[i][0]) # checks against the previous value
            test2 = ((checkedCatch == force_array_seq[i][1]) and (checkedCatch is 1))

            if test2:
                catch_limiter = catch_limiter + 1
            else:
                catch_limiter = 0

            if test == False and catch_limiter < catch_limit:
                checkedNum = force_array_seq[i][0] # if not same then advance to check the next one
                checkedCatch = force_array_seq[i][1]
            else:
                reset = True # if not use the reset to indicate we have to reshuffle.
                #print('reshuffling')
                break

    return force_array_tup

def populate_and_randomize_test(force_array, num_trials):
    '''
    This function populates a test sequence of forces based on the forces specified in force_array with multiples of num_trials.
    The function shuffles the sequence of forces and makes sure that no test forces is repeated back to back.
    Input: force_array (1xN list) , num_trials (integer)
    Output: force_array_seq (1x(N*num_trials) list)
    '''

    force_array_seq = np.zeros(num_trials * len(force_array)) # initialize random sequence list

    idx = 0
    for i in range(num_trials * len(force_array)): # populate the sequence list with the forces
        force_array_seq[i] = force_array[idx]
        if (i + 1) % num_trials == 0:
            idx += 1

    reset = True # this switch defines if we still have to shuffle

    while reset == True:
        reset = False
        np.random.shuffle(force_array_seq)
        checkedNum = force_array_seq[0]

        for i in range(1, num_trials * len(force_array)):
            test = (checkedNum == force_array_seq[i]) # checks against the previous value

            if test == False:
                checkedNum = force_array_seq[i] # if not same then advance to check the next one
            else:
                reset = True # if not use the reset to indicate we have to reshuffle.
                break

    return force_array_seq

def post_trial_feedback(ref_force_current, ref_force_next, act_force, trial_num,feedback_file):
    '''
    This function takes in the current reference force and compares it to the actual force exerted by the user.
    the feedback_file contains the data for the upper and lower error bounds what is considered a "successful" trial
    Input: ref_force_current (double), ref_force_next (double), act_force (double) , trial_num (int) , feedback_file (string)
    Output: msg (ROS message as a string)
    '''

    force_array = np.loadtxt(feedback_file,delimiter=',')
    upper_bounds = force_array[1,:]
    lower_bounds = force_array[2,:]
    ref_force_array = force_array[0,:]

    '''
    upper_bounds = np.array([1.08960022, 1.59593398, 2.6512567, 4.34188334, 6.63222073])
    ref_force_array = np.array([1, 1.5, 2.5, 4, 6])
    lower_bounds = np.array([0.90989649, 1.40609582, 2.36108062, 3.7005955, 5.44771846])
    '''

    ''' Enable this to get qualitative feedback '''
    for i in range(len(ref_force_array)):
        if ref_force_current == ref_force_array[i]:
            
            if act_force[0] < lower_bounds[i]:
                msg = "TOO LOW"
            elif act_force[0] > upper_bounds[i]:
                msg = "TOO HIGH"
            else:
                msg = "CORRECT!"
    
    
    error = round(act_force[0] - ref_force_current,2)
    msg = msg + ', Error: ' + str(error)
            
    msg = 'completed ' + str(trial_num) + '. ' + msg + '. next force : ' + str(ref_force_next)
    return msg

def load_manipulator_pose(filename):

    '''
    This function loads a PyKDL pose from a text file
    Input: filename (string)
    Output: Frame (pyKDL pose)
    '''

    data = np.loadtxt(filename,delimiter=',')
    Rot = PyKDL.Rotation()
    Rot = Rot.Quaternion(data[3], data[4], data[5], data[6])
    Pos = PyKDL.Vector(data[0],data[1],data[2])
    Frame = PyKDL.Frame(Rot,Pos)

    return Frame


class arm_capture_obj:
    '''Object that intializes the manipulators and contains methods for commanding them and recording data'''

    def __init__(self, subj_data):

        self.p2 = dvrk.psm('PSM2')
        self.m2 = dvrk.mtm('MTMR')
        self.m2.set_wrench_body_orientation_absolute(True)
        self.c = dvrk.console()

        self.robot_state = False  # initialize the flag that helps with switch the robot state

        filename = 'Subj' + str(subj_data[0])

        if subj_data[1] == 0:
            filename = filename + '_nohaptics'
        elif subj_data[1] == 1:
            filename = filename + '_haptics'
        else:
            filename = filename + '_manual'

        if subj_data[2] == 0:
            filename = filename + '_train'
        elif subj_data[2] == 1:
            filename = filename + '_test'
        elif subj_data[2] == 2:
            filename = filename + '_rotated'
        elif subj_data[2] == 3:
            filename = filename + '_catch'
        else:
            filename = filename + '_palpate'

        if subj_data[3] == 0:
            filename = filename + '_ef50'
        elif subj_data[3] == 1:
            filename = filename + '_ds10'
        elif subj_data[3] == 2:
            filename = filename + '_ef30'
        else:
            filename = filename + '_ds30'

        self.name = filename

    def set_home_MTM(self, pykdlframe):
        '''
        assigns the mtm seed position for home location
        Input: pykdlframe (pyKDL frame)
        '''
        self.MTMR_pos = pykdlframe

    def set_home_PSM(self, pykdlframe):
        '''
        assigns the psm seed position for home location
        Input: pykdlframe (pyKDL frame)
        '''
        self.PSM_pos = pykdlframe
    
    def home_no_zero(self):
        ''' home the MTMs and PSMs'''
        self.c.teleop_stop()
        print("homing MTM and PSM")
        self.p2.close_jaw()
        self.action_complete = self.p2.move(self.PSM_pos)
        self.m2.move(self.MTMR_pos)
        rospy.sleep(0.5)
    
    def home_all(self,rotation_flag):
        
        self.home_no_zero()
        
        if rotation_flag:
            self.zero_forces_rotated(0.05)
            rospy.sleep(0.25)
            self.zero_forces_rotated(0.05)
        else:
            self.zero_forces(0.05)
            rospy.sleep(0.25)
            self.zero_forces(0.05)
            
        self.c.teleop_start()

    def get_cartesian(self, pose):
        '''
        Takes a pyKDL pose and parses it into cartesian position
        Input: pose (pyKDL pose)
        Output: output (1x3 np array)
        '''

        position = pose.p
        x = position.x()
        y = position.y()
        z = position.z()
        output = np.array([x, y, z])
        return output

    def init_data(self, forcefeedback, EPpose, trial_num):
        '''
        Initialize our data frame
        Input : forcefeedback (1x3 list) , trial_num (int)
        Output: void
        '''

        self.pose_current = self.p2.get_current_position()
        self.pose_desired = self.p2.get_desired_position()
        self.wrench = self.p2.get_current_wrench_body()
        self.force = forcefeedback
        self.time_start = rospy.get_time()  # this re-initializes the start time for each trial
        self.time = rospy.get_time() - self.time_start
        self.pos_current = self.get_cartesian(self.pose_current)
        self.pos_desired = self.get_cartesian(self.pose_desired)
        self.pose_ep = EPpose
        self.ref_force = 0
        self.trial_num = trial_num

        self.data = np.hstack((trial_num, self.ref_force, self.time, self.pose_ep, self.pos_current, self.pos_desired, self.wrench, self.force))

    def record_data(self, forcefeedback, EPpose, ref_force, trial_num):
        '''
        Records data of manipulator pose, experiment conditions and force feedback into an array
        Input : forcefeedback (1x3 list), ref_force (double), trial_num (int)
        Output: returns time? why?
        '''

        self.pose_current = self.p2.get_current_position()
        self.pose_desired = self.p2.get_desired_position()
        self.wrench = self.p2.get_current_wrench_body()
        self.force = forcefeedback
        self.time = rospy.get_time() - self.time_start
        self.pos_current = self.get_cartesian(self.pose_current)
        self.pos_desired = self.get_cartesian(self.pose_desired)
        self.pose_ep = EPpose
        self.ref_force = ref_force
        self.trial_num = trial_num
        new_data = np.hstack(
            (self.trial_num, self.ref_force, self.time, self.pose_ep, self.pos_current, self.pos_desired, self.wrench, self.force))

        # print(new_data)
        # os.system('clear')

        self.data = np.vstack((self.data, new_data))

        return self.time

    def save_data(self):
        '''This method just overwrites the old file with the updated data.
        It should be called after every trial as this way we don't lose any data.'''
        save_filename = self.name + '.csv'
        
        #check if file exists
        if os.path.exists(save_filename):
            print('file already exists. appending...')
            f = open(save_filename,'ab')
        else:
            f = open(save_filename,'wb')
        
        print ('saving ' + save_filename + '...')
        np.savetxt(f, self.data, delimiter=',', fmt='%.4f')
        f.close()
        

    def render_force_feedback(self, force, state_trigger):
        '''The state trigger should be tied to the teleoperation switch on the robot.
        If the trigger is False then we use a dummy dmove to force the robot into the position control mode.'''
        if state_trigger == True:
            self.m2.set_wrench_body_force(force)

            if self.robot_state == False:
                self.robot_state = True

        else:

            if self.robot_state == True:
                self.m2.dmove(PyKDL.Vector(0.0, 0.0, 0.0))
                self.robot_state = False

    def zero_forces(self,epsilon):
        '''
        This function implements a regulator that attempts to drive the forces measured at the force sensor to zero.
        It has use a median filter to try and eliminate noise and some of the viscoelastic effects of the material.
        Input:
        epsilon     The convergence threshold for error.
        '''

        home = False
        Kp = 0.007
        Kd = 0.005

        F_old = force_feedback

        F_array = npm.repmat(force_feedback,10,1)

        while home == False:

            Fx = force_feedback[0]
            Fy = force_feedback[1]
            Fz = force_feedback[2]
            Fx_d = Fx-F_old[0]
            Fy_d = Fy-F_old[1]
            Fz_d = Fz-F_old[2]

            F_old = [Fx,Fy,Fz]

            F_array[0,:] = force_feedback
            F_array[1,:] = F_old
            F_array[2:-1,:] = F_array[1:-2,:]

            #F_average = (np.array(force_feedback) + np.array(F_old)+ np.array(F1)+np.array(F2)+np.array(F3)+np.array(F4))/6
            F_median = np.median(F_array,0)
            F_average = np.mean(F_array,0)

            if np.linalg.norm(F_median)>epsilon:
                self.p2.dmove(PyKDL.Vector(Kp*Fx+Kd*Fx_d, Kp*Fy+Kd*Fy_d, Kp*Fz+Kd*Fz_d))
                #print(np.linalg.norm(F_median))
                #print(np.linalg.norm(force_feedback))
                #print(force_feedback)
                #print(str(Kp*Fx+Kd*Fx_d) + ',' +str(Kp*Fy+Kd*Fy_d) + ',' +str(-(Kp*Fz+Kd*Fz_d)))
            else:
                print(F_median)
                print(F_average)
                home = True

    def zero_forces_rotated(self,epsilon):
        '''
        Same zero_forces but with a transform of 40 degrees implemented to handle the case when the sample is rotated.
        :param epsilon:     convergence threshold
        :return:
        '''

        home = False
        Kp = 0.001
        Kd = 0.00075

        F_old = force_feedback

        F_array = npm.repmat(force_feedback,10,1)

        while home == False:

            Fx = force_feedback[0]
            Fy = force_feedback[1]
            Fz = force_feedback[2]
            Fx_d = Fx-F_old[0]
            Fy_d = Fy-F_old[1]
            Fz_d = Fz-F_old[2]

            theta = 40/180*np.pi # this sets the angle that the sample is rotated

            F_old = [Fx,Fy,Fz]

            F_array[0,:] = force_feedback
            F_array[1,:] = F_old
            F_array[2:-1,:] = F_array[1:-2,:]

            #F_average = (np.array(force_feedback) + np.array(F_old)+ np.array(F1)+np.array(F2)+np.array(F3)+np.array(F4))/6
            F_median = np.median(F_array,0)
            F_average = np.mean(F_array,0)

            if np.linalg.norm(F_median)>epsilon:
                self.p2.dmove(PyKDL.Vector(Kp*(Fx*np.cos(theta)+Fy*np.sin(theta))+Kd*(Fx_d*np.cos(theta)+Fy_d*np.sin(theta)), Kp*(-Fx*np.sin(theta)+Fy*np.cos(theta))+Kd*(-Fx_d*np.sin(theta)+Fy_d*np.cos(theta)), Kp*Fz+Kd*Fz_d))
                #print(np.linalg.norm(F_median))
                #print(np.linalg.norm(force_feedback))
                #print(force_feedback)
                #print(str(Kp*Fx+Kd*Fx_d) + ',' +str(Kp*Fy+Kd*Fy_d) + ',' +str(-(Kp*Fz+Kd*Fz_d)))
            else:
                print(F_median)
                print(F_average)
                home = True

    def zero_force_manually(self,epsilon):

        home = False

        F_old = force_feedback

        F_array = npm.repmat(force_feedback,10,1)

        while home == False:
            Fx = force_feedback[0]
            Fy = force_feedback[1]
            Fz = force_feedback[2]
            Fx_d = Fx-F_old[0]
            Fy_d = Fy-F_old[1]
            Fz_d = Fz-F_old[2]

            F_old = [Fx,Fy,Fz]

            F_array[0,:] = force_feedback
            F_array[1,:] = F_old
            F_array[2:-1,:] = F_array[1:-2,:]

            #F_average = (np.array(force_feedback) + np.array(F_old)+ np.array(F1)+np.array(F2)+np.array(F3)+np.array(F4))/6
            F_median = np.median(F_array,0)
            F_average = np.mean(F_array,0)

            if np.linalg.norm(F_median)>epsilon:
                pass
            else:
                print(F_median)
                print(F_average)
                home = True

class console_capture_obj:

    def __init__(self, subj_data):

        self.c = dvrk.console()

        filename = 'Subj' + str(subj_data[0])

        if subj_data[1] == 0:
            filename = filename + '_nohaptics'
        elif subj_data[1] == 1:
            filename = filename + '_haptics'
        else:
            filename = filename + '_manual'

        if subj_data[2] == 0:
            filename = filename + '_train'
        else:
            filename = filename + '_test'
        if subj_data[3] == 0:
            filename = filename + '_ef50'
        elif subj_data[3] == 1:
            filename = filename + '_ds10'
        elif subj_data[3] == 2:
            filename = filename + '_ef30'
        else:
            filename = filename + '_ds30'

        self.name = filename
        self.action_complete = False

    def init_data(self, forcefeedback, EPpose, trial_num):
        '''
        Initialize our data frame
        Input : forcefeedback (1x3 list) , trial_num (int)
        Output: void
        '''

        self.wrench = np.zeros(6)
        self.force = forcefeedback
        self.time_start = rospy.get_time()  # this re-initializes the start time for each trial
        self.time = rospy.get_time() - self.time_start
        self.pos_current = np.zeros(3)
        self.pos_desired = np.zeros(3)
        self.pose_ep = EPpose
        self.ref_force = 0
        self.trial_num = trial_num

        self.data = np.hstack((trial_num, self.ref_force, self.time, self.pose_ep, self.pos_current, self.pos_desired, self.wrench, self.force))

    def record_data(self, forcefeedback, EPpose, ref_force, trial_num):
        '''
        Records data of manipulator pose, experiment conditions and force feedback into an array
        Input : forcefeedback (1x3 list), ref_force (double), trial_num (int)
        Output: returns time? why?
        '''

        self.wrench = np.zeros(6)
        self.force = forcefeedback
        self.time = rospy.get_time() - self.time_start
        self.pos_current = np.zeros(3)
        self.pos_desired = np.zeros(3)
        self.pose_ep = EPpose
        self.ref_force = ref_force
        self.trial_num = trial_num
        new_data = np.hstack(
            (self.trial_num, self.ref_force, self.time, self.pose_ep, self.pos_current, self.pos_desired, self.wrench, self.force))

        # print(new_data)
        # os.system('clear')

        self.data = np.vstack((self.data, new_data))

        return self.time

    def save_data(self):
        '''This method just checks if the file exists and if so appends the old file with the updated data.
        It should be called after every trial as this way we don't lose any data.'''
        save_filename = self.name + '.csv'
        
        #check if file exists
        if os.path.exists(save_filename):
            f = open(save_filename,'ab')
        else:
            f = open(save_filename,'wb')
        
        print ('saving ' + save_filename + '...')
        np.savetxt(f, self.data, delimiter=',', fmt='%.4f')

    def zero_force_manually(self,epsilon):
        home = False

        F_old = force_feedback

        F_array = npm.repmat(force_feedback,10,1)

        while home == False:
            Fx = force_feedback[0]
            Fy = force_feedback[1]
            Fz = force_feedback[2]
            Fx_d = Fx-F_old[0]
            Fy_d = Fy-F_old[1]
            Fz_d = Fz-F_old[2]

            F_old = [Fx,Fy,Fz]

            F_array[0,:] = force_feedback
            F_array[1,:] = F_old
            F_array[2:-1,:] = F_array[1:-2,:]

            #F_average = (np.array(force_feedback) + np.array(F_old)+ np.array(F1)+np.array(F2)+np.array(F3)+np.array(F4))/6
            F_median = np.median(F_array,0)
            F_average = np.mean(F_array,0)

            if np.linalg.norm(F_median)>epsilon:
                pass
            else:
                print(F_median)
                print(F_average)
                home = True


"""-------------PLEASE PRE-CONFIGURE THESE BEFORE DOING EXPERIMENTS--------------------"""
'''
DEPRACATED
""" MTM home position """
MTMR_cart = PyKDL.Vector(0.055288515671, -0.0508310176185, -0.0659661913251)
MTMR_rot = PyKDL.Rotation()
MTMR_rot = MTMR_rot.Quaternion(0.750403138242, -0.0111643539824, 0.657383142871, -0.0679550644629)
MTMR_pos = PyKDL.Frame(MTMR_rot, MTMR_cart)

""" PSM home position """

PSM_cart = PyKDL.Vector(0.148371870889, -0.0667516027531, -0.0900674974614)
PSM_rot = PyKDL.Rotation()
PSM_rot = PSM_rot.Quaternion(0.747009158404, -0.078584309233, 0.651243196198, -0.10809312193)
PSM_pos = PyKDL.Frame(PSM_rot, PSM_cart)
'''

"""-------------------------------------------------------------------------------------"""

# define our flags
trigger = False  # utility trigger boolean
teleop = False  # teleoperation flag
flag_next = False  # create a flag variable to indicate moving to the next trial (this helps with debouncing)

force_feedback = [0, 0, 0]  # initialize our force_feedback variable
ep_pose = [0,0,0,0,0,0,0]

def main():
    '''MAIN ROUTINE'''
    
    exiter = False  # exit the loop flag

    # collect the filename parameters to initialize the save function in the arm_capture_obj class
    file_data = collect_filename()
    #file_data = np.array([100,1,1,1])
    # 0 for no haptic condition, 1 for haptics, 2 for manual haptics ,
    # 0 for training, 1 for test, 2 for rotation test, 3 for catch, 4 for palpate:

    if file_data[1] == 2 and file_data[2] == 0: # indicates manual training stage
        dvrk_right = console_capture_obj(file_data)
    else:
        # initialize our arm_object
        dvrk_right = arm_capture_obj(file_data)

    # set our script rate
    rate = rospy.Rate(1000)

    # initialize trial number
    #trial_num = 1
    trial_num = input('Key in the trial number you want to start from: ')
    trial_num = trial_num-1

    # create the subscriber to check the footpedals
    sub = rospy.Subscriber('/dvrk/footpedals/camera', Joy, trigger_callback)
    #sub = rospy.Subscriber('/advance_trial', Bool, trigger_callback2)
    teleop_sub = rospy.Subscriber('/dvrk/footpedals/coag', Joy, teleop_callback)
    force_sub = rospy.Subscriber('/force_sensor', Wrench, haptic_feedback)
    ep_sub = rospy.Subscriber('/ep_pose', Pose, EP_pose)
    message_pub = rospy.Publisher('force_msg', String, queue_size=10)
    cam_reset_pub = rospy.Publisher('cam_reset', Bool, queue_size=10)

    '''------------ Loading manipulator home positions ------------'''
    if file_data[2] == 4: # if we are in palpation
        PSM_pos = load_manipulator_pose('./manipulator_homing/psm_home_palp.txt')
        MTMR_pos = load_manipulator_pose('./manipulator_homing/mtm_home_palp.txt')
        dvrk_right.set_home_MTM(MTMR_pos)
        dvrk_right.set_home_PSM(PSM_pos)
        dvrk_right.m2.set_wrench_body_orientation_absolute(True)
    elif file_data[2] == 2: # if we are in rotated testing
        PSM_pos = load_manipulator_pose('./manipulator_homing/psm_home_rot.txt')
        MTMR_pos = load_manipulator_pose('./manipulator_homing/mtm_home_rot.txt')
        dvrk_right.set_home_MTM(MTMR_pos)
        dvrk_right.set_home_PSM(PSM_pos)
        dvrk_right.m2.set_wrench_body_orientation_absolute(True)
    elif file_data[1] == 0 or file_data[1] == 1: # if we are in RMIS
        PSM_pos = load_manipulator_pose('./manipulator_homing/psm_home.txt')
        MTMR_pos = load_manipulator_pose('./manipulator_homing/mtm_home.txt')
        dvrk_right.set_home_MTM(MTMR_pos)
        dvrk_right.set_home_PSM(PSM_pos)
        dvrk_right.m2.set_wrench_body_orientation_absolute(True)
    else: # if we are in manual
        if file_data[2] == 1 or file_data[2] == 3: # if manual testing
            PSM_pos = load_manipulator_pose('./manipulator_homing/psm_home.txt')
            MTMR_pos = load_manipulator_pose('./manipulator_homing/mtm_home.txt')
            dvrk_right.set_home_MTM(MTMR_pos)
            dvrk_right.set_home_PSM(PSM_pos)
            print('set')
            dvrk_right.m2.set_wrench_body_orientation_absolute(True)

    '''------------ Experiment Parameters ------------'''
    num_training_trials = 30 # num trial per training reference force
    break_trial = 30 # num trials before break
    num_test_trials = 5 # num trials per testing reference force
    num_test_trials_gen = 5 # num trial per generalized testing reference force
    num_catch_trials = 2 # num catch trials
    num_consec_catches = 2 # num allowed consecutive catch trials
    default_scale = 0.5 # default teleop scale
    catch_scale = 0.4 # catch trial teleop scale
    countdown_time = 3 # count down time length
    trial_time = 7 # trial time length
    
    # ref_force_array_train = np.array([1,1.5,2.5,4,6])
    # ref_force_array_test = np.array([2,3,4.5,5.5,8])
    
    ref_force_array_train = np.array([1.5,3.5,6,4.5,2.5]) # make sure to staircase it
    ref_force_array_test = np.array([0.75,1,2,3,4,5,7,8])
    ref_force_array_rot = np.array([1, 3, 5, 8])
    ref_force_array_palp = np.array([1, 3, 5, 8])
    ref_force_train = populate_training(ref_force_array_train, num_training_trials)

    if file_data[2] == 3:
        (ref_force_test, ref_force_catch) = populate_and_randomize_test_catch(ref_force_array_test, num_test_trials, num_catch_trials, num_consec_catches) # catch trial function
    elif file_data[2] == 1:
        ref_force_test = populate_and_randomize_test(ref_force_array_test, num_test_trials) # no catch trials
    elif file_data[2] == 2: # rotated
        ref_force_test = populate_and_randomize_test(ref_force_array_rot, num_test_trials_gen)
    else: # palpate
        ref_force_test = populate_and_randomize_test(ref_force_array_palp, num_test_trials_gen)

    # save our experiment sequence data in case something goes wrong and we need to re-run
    if trial_num > 0: # load the files
        if file_data[2] == 0:
            save_filename = dvrk_right.name + 'train_array' + '.csv'
            ref_force_train =  np.loadtxt(save_filename,delimiter=',')
        elif file_data[2] == 1:
            save_filename = dvrk_right.name + 'test_array' + '.csv'
            ref_force_test =  np.loadtxt(save_filename,delimiter=',')
        elif file_data[2] == 2:
            save_filename = dvrk_right.name + 'rot_array' + '.csv'
            ref_force_test =  np.loadtxt(save_filename,delimiter=',')
        elif file_data[2] == 3:
            save_filename = dvrk_right.name + 'catch_array' + '.csv'
            ref_force_test, ref_force_catch =  np.loadtxt(save_filename,delimiter=',')
        else:
            save_filename = dvrk_right.name + 'palp_array' + '.csv'
            ref_force_test =  np.loadtxt(save_filename,delimiter=',')
    else:
        if file_data[2] == 0:
            save_filename = dvrk_right.name + 'train_array' + '.csv'
            np.savetxt(save_filename, ref_force_train, delimiter=',', fmt='%.4f')
        elif file_data[2] == 1:
            save_filename = dvrk_right.name + 'test_array' + '.csv'
            np.savetxt(save_filename, ref_force_test, delimiter=',', fmt='%.4f')
        elif file_data[2] == 2:
            save_filename = dvrk_right.name + 'rot_array' + '.csv'
            np.savetxt(save_filename, ref_force_test, delimiter=',', fmt='%.4f')
        elif file_data[2] == 3:
            save_filename = dvrk_right.name + 'catch_array' + '.csv'
            np.savetxt(save_filename, np.vstack((ref_force_test,ref_force_catch)), delimiter=',', fmt='%.4f')
        else:
            save_filename = dvrk_right.name + 'palp_array' + '.csv'
            np.savetxt(save_filename, ref_force_test, delimiter=',', fmt='%.4f')

    print(ref_force_train)
    print(ref_force_test)

    if file_data[2] == 3:
        print(ref_force_catch)

    # initialize the data structs for recording
    force = [0, 0, 0]
    EPpose = [0,0,0,0,0,0,0]
    dvrk_right.init_data(force,EPpose, trial_num)

    '''
    -------------------------------------------------------------
    ------------------- Experiment Loop -------------------------
    -------------------------------------------------------------
    
    '''
    
    while exiter == False and not rospy.is_shutdown():
    
        dvrk_right.c.set_teleop_scale(default_scale)
        trial_num += 1  # increment our trial num
        flag_next = False  # reset our flag next

        ''' Homing Sequence '''
        if file_data[2] == 4: # is we are in palpation home without force zeroing
            dvrk_right.home_no_zero()
            dvrk_right.c.teleop_start()
            #while dvrk_right.action_complete == False:
                #print(dvrk_right.action_complete)
        elif file_data[2] == 2:
            print('Homing manipulators... \n')
            dvrk_right.home_all(True) # home all with rotation flag set to True
            #while dvrk_right.action_complete == False:
                #print(dvrk_right.action_complete)
        elif file_data[1] == 0 or file_data[1] == 1: # only do auto homing if the experiment condition is teleoperated
            print('Homing manipulators... \n')
            dvrk_right.home_all(False)
            #while dvrk_right.action_complete == False:
                #print(dvrk_right.action_complete)
        else:
            if file_data[2] == 0: #if we are in manual training don't home. Let the user just reset themselves
                print('Waiting for user to reset...\n')
                dvrk_right.zero_force_manually(0.075) # this epsilon needs to be tuned for manual ability
                dvrk_right.action_complete = True
            else: # if we are in manual testing, then we still have to do homing.
                print('Homing manipulators... \n')
                dvrk_right.home_all(False)
                #while dvrk_right.action_complete == False:
                    #print(dvrk_right.action_complete)
        
        if (trial_num)%break_trial==1 and file_data[2] == 0 and trial_num>break_trial: # if we are in training enforce the breaks
                continue_flag = False
                while continue_flag != 1:
                    message_pub.publish('Well done :) it is break time')
                    continue_flag = input("Break Time. Once ready, enter 1 to continue: ")
                dvrk_right.time_start = rospy.get_time() # reset our timer
        
        #print('Homing Complete: ' + str(dvrk_right.action_complete))
        cam_reset_pub.publish(True)
        countdown = True
        if file_data[2] == 0:
            if not trial_num > len(ref_force_train):
                if countdown:
                    count_time = rospy.get_time()
                    count_down = False
                while (rospy.get_time() - count_time) <= 3: # countdown timer is set to 3s
                    message_pub.publish('Begin in %.0fs! Target: %.2f ' % (3-(rospy.get_time()-count_time),ref_force_train[trial_num - 1]))
                    #dvrk_right.c.teleop_stop()
                message_pub.publish('Go!!!')
            else:
                message_pub.publish('End!')
        else:
            if not trial_num > len(ref_force_test): 
                if countdown:
                    count_time = rospy.get_time()
                    count_down = False
                while (rospy.get_time() - count_time) <= 3:
                    message_pub.publish('Begin in %.0fs! Target: %.2f ' % (3-(rospy.get_time()-count_time),ref_force_test[trial_num - 1]))
                    #dvrk_right.c.teleop_stop()
                message_pub.publish('Go!!!')
            else:
                message_pub.publish('End!')
                
        dvrk_right.c.teleop_start()        
        dvrk_right.action_complete = False  # reset our flag

        dvrk_right.time_start = rospy.get_time() # reset our timer

        '''
        /////////////////////////////////////////////////////////////////////////////////
        ////////////////////////// Training Phase with No Haptics ///////////////////////
        ////////////////////////////////////////////////////////////////////////////////
        '''

        if file_data[1] == 0 and file_data[2] == 0:

            # check if we are at the end of our test condition and if we are we flip the exiter flag
            if trial_num == len(ref_force_train)+1:
                exiter = True
                break

            print('Starting Trial for Training, No Haptics, Trial No. ' + str(trial_num) + ', Force Level: ' + str(ref_force_train[trial_num-1]))

            while flag_next == False and not rospy.is_shutdown():
                              
                force = force_feedback  # collect force data from sensor
                EPpose = ep_pose # collect end effector pose from sensor
                
                time = dvrk_right.record_data(force, EPpose, ref_force_train[trial_num - 1], trial_num)
                #message_pub.publish('%.1fs' % time)
                if (time > 0.5 and flag_next == False and trigger == True):
                    flag_next = True
                    if trial_num < len(ref_force_train):
                        message = post_trial_feedback(ref_force_train[trial_num - 1], ref_force_train[trial_num], force,
                                                  trial_num,'force_bounds.csv')
                        message_pub.publish(message)
                    else:
                        message = post_trial_feedback(ref_force_train[trial_num - 1], 0, force,
                                                  trial_num,'force_bounds.csv')
                        message_pub.publish(message)                    

                rate.sleep()

            # END OF WHILE LOOP

        '''
        /////////////////////////////////////////////////////////////////////////////////
        ////////////////////////// Training Phase with Haptics //////////////////////////
        /////////////////////////////////////////////////////////////////////////////////
        '''

        if file_data[1] == 1 and file_data[2] == 0:

            # check if we are at the end of our test condition and if we are we flip the exiter flag
            if trial_num == len(ref_force_train)+1:
                exiter = True
                break

            print('Starting Trial for Training, with Haptics, Trial No. ' + str(trial_num) + ', Force Level: ' + str(ref_force_train[trial_num-1]) )

            while flag_next == False and not rospy.is_shutdown():
                                
                force = force_feedback  # collect force data from sensor
                EPpose = ep_pose # collect end effector pose from sensor
                dvrk_right.render_force_feedback(force, teleop) # apply for force feedback to MTM
                time = dvrk_right.record_data(force, EPpose, ref_force_train[trial_num - 1], trial_num)
                
                #if time>trial_time:
                #    print('times up!!!!')
                #message_pub.publish('%.1fs' % time)
                
                if (time > 0.5 and flag_next == False and trigger == True):
                    flag_next = True
                    if trial_num < len(ref_force_train):
                        message = post_trial_feedback(ref_force_train[trial_num - 1], ref_force_train[trial_num], force, trial_num,'force_bounds.csv')
                        message_pub.publish(message)
                    else:
                        message = post_trial_feedback(ref_force_train[trial_num - 1], 0, force,
                                                  trial_num,'force_bounds.csv')
                        message_pub.publish(message) 
                        
                rate.sleep()

            # END OF WHILE LOOP

        '''
        /////////////////////////////////////////////////////////////////////////////////
        /////////////////// Training Phase with Manual Haptics //////////////////////////
        /////////////////////////////////////////////////////////////////////////////////
        '''
        if file_data[1] == 2 and file_data[2] == 0:

            # check if we are at the end of our test condition and if we are we flip the exiter flag
            if trial_num == len(ref_force_train)+1:
                exiter = True
                break

            print('Starting Trial for Training, with Manual Haptics, Trial No. ' + str(trial_num) + ', Force Level: ' + str(ref_force_train[trial_num-1]) )

            while flag_next == False and not rospy.is_shutdown():
                     
                force = force_feedback  # use the force feedback
                EPpose = ep_pose
                
                time = dvrk_right.record_data(force, EPpose, ref_force_train[trial_num - 1], trial_num)
                #message_pub.publish('%.1fs' % time)
                if (time > 0.5 and flag_next == False and trigger == True):
                    flag_next = True
                    if trial_num < len(ref_force_train):
                        message = post_trial_feedback(ref_force_train[trial_num - 1], ref_force_train[trial_num], force, trial_num,'force_bounds.csv')
                        message_pub.publish(message)
                    else:
                        message = post_trial_feedback(ref_force_train[trial_num - 1], 0, force,
                                                  trial_num,'force_bounds.csv')
                        message_pub.publish(message)

                rate.sleep()

            # END OF WHILE LOOP

        '''
        /////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////// Test Phase ///////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////
        '''

        if file_data[2] == 1 or file_data[2] == 3:

            # check if we are at the end of our test condition and if we are we flip the exiter flag
            if trial_num == len(ref_force_test)+1:
                exiter = True
                break

            if file_data[1] == 1:
                print('Starting Trial for Test, Training with Haptics, Trial No. ' + str(trial_num) + ', Force Level: ' + str(ref_force_test[trial_num-1]))
            elif file_data[1] == 2:
                print('Starting Trial for Test, Training with no Haptics, Trial No. ' + str(trial_num) + ', Force Level: ' + str(ref_force_test[trial_num-1]))
            else:
                print('Starting Trial for Test, Training with Manual Haptics, Trial No. ' + str(trial_num) + ', Force Level: ' + str(ref_force_test[trial_num-1]))

            ''' Catch Trial Conditional Statement'''
            if file_data[2] == 3:
                print('catch trials enabled')
                if ref_force_catch[trial_num-1] == 1:
                    print('this trial is catch')
                    dvrk_right.c.set_teleop_scale(catch_scale)
                else:
                    print('this trial in not catch')
                    dvrk_right.c.set_teleop_scale(default_scale)
 
            while flag_next == False and not rospy.is_shutdown():

                force = force_feedback  # use the force feedback
                EPpose = ep_pose
                    
                time = dvrk_right.record_data(force, EPpose, ref_force_test[trial_num - 1], trial_num)
                #message_pub.publish('%.1fs' % time)                
                if (time > 0.5 and flag_next == False and trigger == True):
                    flag_next = True
                    if trial_num < len(ref_force_test):
                        message = 'next target force: ' + str(ref_force_test[trial_num])
                        message_pub.publish(message)

                rate.sleep()

        '''
        /////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////// Test Phase Rotated //////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////
        '''

        if file_data[2] == 2:

            # check if we are at the end of our test condition and if we are we flip the exiter flag
            if trial_num == len(ref_force_test)+1:
                exiter = True
                break

            if file_data[1] == 1:
                print('Starting Trial for GTest, Training with Haptics, Trial No. ' + str(trial_num) + ', Force Level: ' + str(ref_force_test[trial_num-1]))
            elif file_data[1] == 2:
                print('Starting Trial for GTest, Training with no Haptics, Trial No. ' + str(trial_num) + ', Force Level: ' + str(ref_force_test[trial_num-1]))
            else:
                print('Starting Trial for GTest, Training with Manual Haptics, Trial No. ' + str(trial_num) + ', Force Level: ' + str(ref_force_test[trial_num-1]))

            while flag_next == False:

                force = force_feedback  # use the force feedback
                EPpose = ep_pose
                time = dvrk_right.record_data(force, EPpose, ref_force_test[trial_num - 1], trial_num)
                #message_pub.publish('%.1fs' % time)                
                
                if (time > 0.5 and flag_next == False and trigger == True):
                    flag_next = True
                    if trial_num < len(ref_force_test):
                        message = 'next target force: ' + str(ref_force_test[trial_num])
                        message_pub.publish(message)

                rate.sleep()

            # END OF WHILE LOOP

        '''
        /////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////// Test Phase Palpation //////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////
        '''

        if file_data[2] == 4:

            # check if we are at the end of our test condition and if we are we flip the exiter flag
            if trial_num == len(ref_force_test)+1:

                exiter = True
                break

            if file_data[1] == 1:
                print('Starting Trial for PalpTest, Training with Haptics, Trial No. ' + str(trial_num) + ', Force Level: ' + str(ref_force_test[trial_num-1]))
            elif file_data[1] == 2:
                print('Starting Trial for PalpTest, Training with no Haptics, Trial No. ' + str(trial_num) + ', Force Level: ' + str(ref_force_test[trial_num-1]))
            else:
                print('Starting Trial for PalpTest, Training with Manual Haptics, Trial No. ' + str(trial_num) + ', Force Level: ' + str(ref_force_test[trial_num-1]))

            while flag_next == False and not rospy.is_shutdown():

                force = force_feedback  # use the force feedback
                EPpose = ep_pose
 
                    
                time = dvrk_right.record_data(force, EPpose, ref_force_test[trial_num - 1], trial_num)
                #message_pub.publish('%.1fs' % time)
                if (time > 0.5 and flag_next == False and trigger == True):
                    flag_next = True
                    if trial_num < len(ref_force_test):
                        message = 'next target force: ' + str(ref_force_test[trial_num])
                        message_pub.publish(message)

                rate.sleep()

            # END OF WHILE LOOP

        ## End of the all the conditionals

        print ('Trial ' + str(trial_num) + ' completed. \n')
        print('Saving data... \n')
        dvrk_right.save_data()
        
        
        # clear the data array and reinitialize
        dvrk_right.data = None
        force = [0, 0, 0]
        EPpose = [0,0,0,0,0,0,0]
        dvrk_right.init_data(force,EPpose, trial_num)
        
        
    print('End of Experiment')


if __name__ == "__main__":
    main()
