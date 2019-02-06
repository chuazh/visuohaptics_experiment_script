#!/usr/bin/env python

""" 
Script to Pilot Test With Manual Haptics
by Zonghe Chua 02/06/19

This script run the pilot test study with manual haptics

It requires initializing the MSM and PSM "home" positions.

"""

import rospy
import dvrk
import numpy as np
import signal
import PyKDL
import os
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3, Quaternion, Wrench
from std_msgs.msg import String, Bool
import numpy.matlib as npm

def trigger_callback(data):
    global trigger
    butt = data.buttons[0]
    if butt > 0.5:
        trigger = True
    else:
        trigger = False
    return


def teleop_callback(data):
    global teleop
    butt = data.buttons[0]
    if butt > 0.5:
        teleop = True
    else:
        teleop = False


def haptic_feedback(data):
    global force_feedback
    force_feedback = [0, 0, 0]
    force_feedback[0] = data.force.x
    force_feedback[1] = data.force.y
    force_feedback[2] = -data.force.z


def collect_filename():
    # This function collects the relevant parameters for our experiment

    subj = input("Please key in subject number: ")
    print '\n'

    haptic = input("Please enter 0 for no haptic condition and 1 for haptics: ")
    print '\n'

    test = input("Please enter 0 for training and 1 for test: ")
    print '\n'

    material_select = input("Please select material. 0 for EF and 1 for DS: ")

    return np.array([subj, haptic, test, material_select])


def populate_training(force_array, num_trials):
    # This function populates a training sequence of forces based on the forces specified in the force_array with a multiple of num_trials for each
    force_array_seq = np.zeros(num_trials * len(force_array))
    idx = 0
    for i in range(num_trials * len(force_array)):
        force_array_seq[i] = force_array[idx]
        if (i + 1) % num_trials == 0:
            idx += 1

    return force_array_seq


def populate_and_randomize_test(force_array, num_trials):
    # This function populates a test sequence of forces based on the forces specified in force_array with multiples of num_trials.
    # The function shuffles the sequence of forces and makes sure that no test forces is repeated back to back.

    force_array_seq = np.zeros(num_trials * len(force_array))
    idx = 0
    for i in range(num_trials * len(force_array)):
        force_array_seq[i] = force_array[idx]
        if (i + 1) % num_trials == 0:
            idx += 1

    reset = True

    while reset == True:
        reset = False
        np.random.shuffle(force_array_seq)
        checkedNum = force_array_seq[0]
        for i in range(1, num_trials * len(force_array)):
            test = (checkedNum == force_array_seq[i])

            if test == False:
                checkedNum = force_array_seq[i]
            else:
                reset = True
                break

    return force_array_seq


def post_trial_feedback(ref_force_current, ref_force_next, act_force, trial_num,feedback_file):
    # This function takes in the current reference force and compares it to the actual force exerted by the user.
    # the feedback_file contains the data for the upper and lower error bounds what is considered a "successful" trial

    force_array = np.loadtxt(feedback_file,delimiter=',')
    upper_bounds = force_array[1,:]
    lower_bounds = force_array[2,:]
    ref_force_array = force_array[0,:]

    '''
    upper_bounds = np.array([1.08960022, 1.59593398, 2.6512567, 4.34188334, 6.63222073])
    ref_force_array = np.array([1, 1.5, 2.5, 4, 6])
    lower_bounds = np.array([0.90989649, 1.40609582, 2.36108062, 3.7005955, 5.44771846])
    '''

    for i in range(len(ref_force_array)):
        if ref_force_current == ref_force_array[i]:

            if act_force[0] < lower_bounds[i]:
                msg = "TOO LOW"
            elif act_force[0] > upper_bounds[i]:
                msg = "TOO HIGH"
            else:
                msg = "CORRECT!"
    msg = 'completed ' + str(trial_num) + '. ' + msg + '. next target force is ' + str(ref_force_next)
    return msg

def load_manipulator_pose(filename):
    data = np.loadtxt(filename,delimiter=',')
    Rot = PyKDL.Rotation()
    Rot = Rot.Quaternion(data[3], data[4], data[5], data[6])
    Pos = PyKDL.Vector(data[0],data[1],data[2])
    Frame = PyKDL.Frame(Rot,Pos)

    return Frame

class arm_capture_obj:
    def __init__(self, subj_data):

        self.p2 = dvrk.psm('PSM2')
        self.m2 = dvrk.mtm('MTMR')
        self.m2.set_wrench_body_orientation_absolute(True)
        self.c = dvrk.console()

        self.robot_state = False  # initialize the flag that helps with switch the robot state

        filename = 'Subj' + str(subj_data[0])

        if subj_data[1] == 0:
            filename = filename + '_nohaptic'
        else:
            filename = filename + '_haptics'

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

    def set_home_MTM(self, pykdlframe):
        self.MTMR_pos = pykdlframe

    def set_home_PSM(self, pykdlframe):
        self.PSM_pos = pykdlframe

    def home_all(self):
        # home the MTMs and PSMs
        self.c.teleop_stop()
        print("homing MTM and PSM")
        self.p2.close_jaw()
        self.action_complete = self.p2.move(self.PSM_pos)
        self.m2.move(self.MTMR_pos)
        rospy.sleep(0.5)
        self.zero_forces(0.01)
        rospy.sleep(0.25)
        self.zero_forces(0.01)
        self.c.teleop_start()

    def get_cartesian(self, pose):
        position = pose.p
        x = position.x()
        y = position.y()
        z = position.z()
        output = np.array([x, y, z])
        return output

    def init_data(self, forcefeedback, trial_num):
        self.pose_current = self.p2.get_current_position()
        self.pose_desired = self.p2.get_desired_position()
        self.wrench = self.p2.get_current_wrench_body()
        self.force = forcefeedback
        self.time_start = rospy.get_time()  # this re-initializes the start time for each trial
        self.time = rospy.get_time() - self.time_start
        self.pos_current = self.get_cartesian(self.pose_current)
        self.pos_desired = self.get_cartesian(self.pose_desired)
        self.ref_force = 0
        self.trial_num = trial_num
        self.data = np.hstack((trial_num, self.ref_force, self.time, self.pos_current, self.pos_desired, self.wrench, self.force))

    def record_data(self, forcefeedback, ref_force, trial_num):
        self.pose_current = self.p2.get_current_position()
        self.pose_desired = self.p2.get_desired_position()
        self.wrench = self.p2.get_current_wrench_body()
        self.force = forcefeedback
        self.time = rospy.get_time() - self.time_start
        self.pos_current = self.get_cartesian(self.pose_current)
        self.pos_desired = self.get_cartesian(self.pose_desired)
        self.ref_force = ref_force
        self.trial_num = trial_num
        new_data = np.hstack(
            (self.trial_num, self.ref_force, self.time, self.pos_current, self.pos_desired, self.wrench, self.force))

        # print(new_data)
        # os.system('clear')

        self.data = np.vstack((self.data, new_data))

        return self.time

    def save_data(self):
        '''This method just overwrites the old file with the updated data.
        It should be called after every trial as this way we don't lose any data.'''
        save_filename = self.name + '.csv'
        print ('saving ' + save_filename + '...')
        np.savetxt(save_filename, self.data, delimiter=',', fmt='%.4f')

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
                self.p2.dmove(PyKDL.Vector(Kp*Fx+Kd*Fx_d, Kp*Fz+Kd*Fz_d, -(Kp*Fy+Kd*Fy_d)))
                #print(np.linalg.norm(F_median))
                #print(np.linalg.norm(force_feedback))
                #print(force_feedback)
                #print(str(Kp*Fx+Kd*Fx_d) + ',' +str(Kp*Fy+Kd*Fy_d) + ',' +str(-(Kp*Fz+Kd*Fz_d)))
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


def main():
    '''MAIN ROUTINE'''

    exiter = False  # exit the loop flag

    # collect the filename parameters to initialize the save function in the arm_capture_obj class
    file_data = collect_filename()
    # file_data = np.array([1,1,0,1])

    # initialize our arm_object
    dvrk_right = arm_capture_obj(file_data)

    # set our script rate
    rate = rospy.Rate(1000)

    # initialize trial number
    #trial_num = 0
    trial_num = input('Key in the trial number you want to start from: ')
    trial_num = trial_num-1

    break_trial = 30

    # create the subscriber to check the footpedals
    sub = rospy.Subscriber('/dvrk/footpedals/camera', Joy, trigger_callback)
    teleop_sub = rospy.Subscriber('/dvrk/footpedals/coag', Joy, teleop_callback)
    force_sub = rospy.Subscriber('/force_sensor', Wrench, haptic_feedback)
    message_pub = rospy.Publisher('force_msg', String, queue_size=10)
    cam_reset_pub = rospy.Publisher('cam_reset', Bool, queue_size=10)

    PSM_pos = load_manipulator_pose('./manipulator_homing/psm_home.txt')
    MTMR_pos = load_manipulator_pose('./manipulator_homing/mtm_home.txt')

    dvrk_right.set_home_MTM(MTMR_pos)
    dvrk_right.set_home_PSM(PSM_pos)

    num_training_trials = 30
    num_test_trials = 3

    # ref_force_array_train = np.array([1,1.5,2.5,4,6])
    ref_force_array_train = np.array([1, 2.5, 5.5, 4, 1.5])
    ref_force_array_test = np.array([0.75, 2, 3, 4.5, 6])
    # ref_force_array_test = np.array([2,3,4.5,5.5,8])
    ref_force_train = populate_training(ref_force_array_train, num_training_trials)
    ref_force_test = populate_and_randomize_test(ref_force_array_test, num_test_trials)

    print(ref_force_train)
    print(ref_force_test)

    force = [0, 0, 0]  # hardset the forces to 0

    dvrk_right.m2.set_wrench_body_orientation_absolute(True)

    dvrk_right.init_data(force, trial_num)

    while exiter == False:

        trial_num += 1  # increment our trial num
        flag_next = False  # reset our flag next

        print('Homing manipulators... \n')
        dvrk_right.home_all()

        while dvrk_right.action_complete == False:
            print(dvrk_right.action_complete)

        print('Homing Complete: ' + str(dvrk_right.action_complete))
        cam_reset_pub.publish(True)
        message_pub.publish('Begin!!')
        dvrk_right.action_complete = False  # reset our flag

        '''Training Phase with No Haptics'''

        if file_data[1] == 0 and file_data[2] == 0:

            # check if we are at the end of our test condition and if we are we flip the exiter flag
            if trial_num == len(ref_force_train)+1:
                exiter = True
                break

            if (trial_num)%break_trial==1 and trial_num>break_trial:
                continue_flag = False
                while continue_flag != 1:
                    continue_flag = input("Break Time. Once ready, enter 1 to continue: ")


            print('Starting Trial for Training, No Haptics, Trial No. ' + str(trial_num) + ', Force Level: ' + str(ref_force_train[trial_num-1]))

            while flag_next == False:

                force = force_feedback  # use the force feedback
                time = dvrk_right.record_data(force, ref_force_train[trial_num - 1], trial_num)

                if time > 0.5 and flag_next == False and trigger == True:
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

        '''Training Phase with Haptics'''

        if file_data[1] == 1 and file_data[2] == 0:

            # check if we are at the end of our test condition and if we are we flip the exiter flag
            if trial_num == len(ref_force_train)+1:
                exiter = True
                break

            if (trial_num)%break_trial==1 and trial_num>break_trial:
                continue_flag = False
                while continue_flag != 1:
                    continue_flag = input("Break Time. Once ready, enter 1 to continue: ")

            print('Starting Trial for Training, with Haptics, Trial No. ' + str(trial_num) + ', Force Level: ' + str(ref_force_train[trial_num-1]) )

            while flag_next == False:

                force = force_feedback  # use the force feedback

                dvrk_right.render_force_feedback(force, teleop)
                time = dvrk_right.record_data(force, ref_force_train[trial_num - 1], trial_num)

                if time > 0.5 and flag_next == False and trigger == True:
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

        '''Test Phase'''

        if file_data[2] == 1:

            # check if we are at the end of our test condition and if we are we flip the exiter flag
            if trial_num == len(ref_force_test)+1:
                exiter = True
                break

            if file_data[1] == 1:
                print('Starting Trial for Test, Training with Haptics, Trial No. ' + str(trial_num) + ', Force Level: ' + str(ref_force_test[trial_num-1]))
            else:
                print('Starting Trial for Test, Training with no Haptics, Trial No. ' + str(trial_num) + ', Force Level: ' + str(ref_force_test[trial_num-1]))

            while flag_next == False:

                force = force_feedback  # use the force feedback
                time = dvrk_right.record_data(force, ref_force_test[trial_num - 1], trial_num)

                if time > 0.5 and flag_next == False and trigger == True:
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

    print('End of Experiment')


if __name__ == "__main__":
    main()
