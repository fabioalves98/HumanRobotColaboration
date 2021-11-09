#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
from statistics import mean

import cobot.helpers as helpers
from cobot.hand_guiding.wrench_theory import theoryFT


JOINTS = [0, -90, -90, -90, 0, 0]
W_2 = [-90, -45, 0, 45, 90]


tests = [
    '/record/last/1_noGripper_0Pay/',
    '/record/last/2_noGripper_15Pay/',
    '/record/last/3_noGripper_15Pay_42Cog/',
    '/record/last/4_noGripper_15Pay_100Cog/',
    '/record/last/5_gripperCoupled_0Pay/',
    '/record/last/6_gripper_0Pay/',
    '/record/last/7_gripper_15Pay/',
    '/record/last/8_gripper_15Pay_42Cog/',
    '/record/last/9_gripper_object_1Kg/',
    '/record/last/10_gripper_object_2Kg/',
    '/record/last/11_z_noGripper_0Pay/',
    '/record/last/12_z_noGripper_15Pay_42Cog/',
    '/record/last/13_z_gripper_0Pay/',
    '/record/last/14_z_gripper_15Pay_42Cog/',
    '/record/last/15_z_gripper_object_1Kg/',
    '/record/last/16_ft_sensor_weight_1Kg/'
]

poses_file = '/record/last/poses.list'

correct_w3 = '/curves/wrench_correct_final_720.list'

def openTests(test_idx, outliers = False):
    samples = []

    # Open all testes inside specific test folder
    for test_file in sorted(os.listdir(helpers.BASE_DIR + tests[test_idx])):
        try:
            sample = helpers.openList(tests[test_idx] + test_file)
            
            # Remove initial outliers
            if outliers:
                for e in range(3):
                    sample[0, 0, e] = sample[2, 0, e]
                    sample[1, 0, e] = sample[2, 0, e]
                    sample[0, 1, e] = sample[2, 1, e]
                    sample[1, 1, e] = sample[2, 1, e]
                
            samples.append(sample)
        
        except IOError:
            print('Test file not found')
        
    return samples


def test0noGripperNoPay():
    samples = openTests(0)
    average = np.empty((5, 720, 2, 3))
    
    # Set plot propertires
    fig, s_plt = plt.subplots(2)
    x = np.arange(-360, 360, 1)
    xlabel = 'Wrist 3 Angle($\degree$)'
    ylabel_f = 'Force(N)'
    ylabel_t = 'Torque(Nm)'
    legend=['x', 'y', 'z']

    # Plot original samples
    for i in range(5):
        average[i] = samples[i]
        helpers.plotXYZ(s_plt[0], x, samples[i][:,0,:], ':', 0.5)
        helpers.plotXYZ(s_plt[1], x, samples[i][:,1,:], ':', 0.5)

    # Compute average
    average = np.mean(average, axis=0)

    # Plot average
    helpers.plotXYZ(s_plt[0], x, average[:,0,:], xlabel=xlabel, ylabel=ylabel_f, legend=legend)
    helpers.plotXYZ(s_plt[1], x, average[:,1,:], xlabel=xlabel, ylabel=ylabel_t, legend=legend)

    plt.show()


def test1to3noGripper(test_idx):
    # Obtain tests and choose specific position(P3)
    samples = openTests(test_idx)
    samples = samples[2:3]
    
    # Set plot properties
    fig, s_plt = plt.subplots(2)
    x = np.arange(-360, 360, 1)
    xl = 'Wrist 3 Angle($\degree$)'
    yl_f = 'Force(N)'
    yl_t = 'Torque(Nm)'
    leg = ['x', 'y', 'z']
    leg_b = [l + '_raw' for l in leg]
    leg_a = [l + '_corrected' for l in leg]

    # Plot original samples
    for sample in samples:
        helpers.plotXYZ(s_plt[0], x, sample[:,0,:], ':', legend=leg_b)
        helpers.plotXYZ(s_plt[1], x, sample[:,1,:], ':', legend=leg_b)

    # Plot correction curve
    correction = helpers.openList(correct_w3)
    # helpers.plotXYZ(s_plt[0], x, correction[:,0,:], '--')
    # helpers.plotXYZ(s_plt[1], x, correction[:,1,:], '--')

    # Plot corrected samples
    for sample in samples:
        sample_c = sample - correction
        helpers.plotXYZ(s_plt[0], x, sample_c[:,0,:], xlabel=xl, ylabel=yl_f, legend=leg_a)
        helpers.plotXYZ(s_plt[1], x, sample_c[:,1,:], xlabel=xl, ylabel=yl_t, legend=leg_a)

    plt.show()


def test4gripperCouple():
    samples = openTests(4)

    fig, s_plt = plt.subplots(2)
    x = np.arange(-360, 360, 1)

    # Plot original samples
    # for sample in samples:
    # helpers.plotXYZ(s_plt[0], x, samples[2][:,0,:], ':')
    # helpers.plotXYZ(s_plt[1], x, samples[2][:,1,:], ':')

    # Plot correction curve
    correction = helpers.openList(correct_w3)
    # helpers.plotXYZ(s_plt[0], x, correction[:,0,:], '--')
    # helpers.plotXYZ(s_plt[1], x, correction[:,1,:], '--')

    # # Plot corrected samples
    # for sample in samples:
    corrected_sample = samples[2] - correction
    helpers.plotXYZ(s_plt[0], x, corrected_sample[:,0,:], title='Force')
    helpers.plotXYZ(s_plt[1], x, corrected_sample[:,1,:], title='Torque')

    plt.show()


def test5gripperNoPay():
    # Obtain tests and choose specific position(P3)
    samples = openTests(5)
    samples = samples[2:3]

    # Set plot properties
    fig, s_plt = plt.subplots(2)
    x = np.arange(-360, 360, 1)
    xl = 'Wrist 3 Angle($\degree$)'
    yl_f = 'Force(N)'
    yl_t = 'Torque(Nm)'
    leg = ['x', 'y', 'z']
    leg_b = [l + '_raw' for l in leg]
    leg_a = [l + '_corrected' for l in leg]

    # Plot original samples
    for sample in samples:
        helpers.plotXYZ(s_plt[0], x, sample[:,0,:], ':', legend=leg_b)
        helpers.plotXYZ(s_plt[1], x, sample[:,1,:], ':', legend=leg_b)

    # Plot correction curve
    correction = helpers.openList(correct_w3)
    # helpers.plotXYZ(s_plt[0], x, correction[:,0,:], '--')
    # helpers.plotXYZ(s_plt[1], x, correction[:,1,:], '--')

    # # Plot corrected samples
    for sample in samples:
        sample_c = sample - correction
        helpers.plotXYZ(s_plt[0], x, sample_c[:,0,:], xlabel=xl, ylabel=yl_f, legend=leg_a)
        helpers.plotXYZ(s_plt[1], x, sample_c[:,1,:], xlabel=xl, ylabel=yl_t, legend=leg_a)

    plt.show()
        

def test8to9object(test_idx):
    samples = openTests(test_idx)

    fig, s_plt = plt.subplots(2)
    x = np.arange(-360, 360, 1)

    # Plot correction curve
    correction = helpers.openList(correct_w3)
    # helpers.plotXYZ(s_plt[0], x, correction[:,0,:], '--')
    # helpers.plotXYZ(s_plt[1], x, correction[:,1,:], '--')

    # # Plot corrected samples
    for sample in samples:
        corrected_sample = sample - correction
        helpers.plotXYZ(s_plt[0], x, corrected_sample[:,0,:], ':', title='Force')
        helpers.plotXYZ(s_plt[1], x, corrected_sample[:,1,:], ':', title='Torque')

    plt.show()


def test12gripperZNoPay():
    joints = JOINTS
    sample_theory = np.empty([360, 2, 3])

    samples = openTests(12, True)

    x = np.arange(-180, 180, 1)

    # Plot original samples
    for i in range(5):
        joints[5] = W_2[i]
        print(joints)
        for w_2 in range(-180, 180, 1):
            joints[4] = w_2
            joints_rad = [math.radians(j) for j in joints]
            pose = helpers.fkService(joints_rad)
            theory = theoryFT(pose.rotation)

            sample_theory[w_2][0] = [theory.force.x, theory.force.y, theory.force.z]
            sample_theory[w_2][1] = [theory.torque.x, theory.torque.y, theory.torque.z]

        # Offset theory test on W_3 = 0 (same as waht happened in real test)
        sample_theory -= sample_theory[180,:,:]

        fig, s_plt = plt.subplots(2)

        # Plot real
        helpers.plotXYZ(s_plt[0], x, samples[i][:,0,:], ':')
        helpers.plotXYZ(s_plt[1], x, samples[i][:,1,:], ':')

        # Plot theory test
        helpers.plotXYZ(s_plt[0], x, sample_theory[:,0,:])
        helpers.plotXYZ(s_plt[1], x, sample_theory[:,1,:])

        plt.show()


def testTheoryModel():
    # Theory Results initiation
    joints = JOINTS
    sample_theory = np.empty([720, 2, 3])

    # Real test and select P3
    samples = openTests(5)
    correction = helpers.openList(correct_w3)

    # Choose betwwen plot or hist
    plot_hist = True

    # Plot parameters
    x = np.arange(-360, 360, 1)
    xl = 'Wrist 3 Angle($\degree$)'
    yl_f = 'Force(N)'
    yl_t = 'Torque(Nm)'
    leg = ['x', 'y', 'z']
    leg_t = [l + '_theory' for l in leg]
    leg_r = [l + '_real' for l in leg]

    # Histogram parametes
    color = ['r', 'g', 'b']
    sample_diff = np.empty([0, 2, 3])

    # Make 5 plots of the 5 test positions
    for i in range(5):
        joints[4] = W_2[i]
        # Create Theory test based on the  desired configuration
        for w_3 in range(-360, 360, 1):
            joints[5] = w_3
            joints_rad = [math.radians(j) for j in joints]
            pose = helpers.fkService(joints_rad)
            theory = theoryFT(pose.rotation)

            sample_theory[w_3][0] = [theory.force.x, theory.force.y, theory.force.z]
            sample_theory[w_3][1] = [theory.torque.x, theory.torque.y, theory.torque.z]

        # Offset theory test on W_3 = 0 (same as waht happened in real test)
        sample_theory -= sample_theory[360,:,:]

        # Obtain real test and correct it
        sample_c = samples[i] - correction

        # Obtain differences betwwen theory model and real
        sample_diff = np.append(sample_diff, np.abs(sample_theory - sample_c), axis=0)

        if plot_hist:
            fig, s_plt = plt.subplots(2)

            # Plot real
            helpers.plotXYZ(s_plt[0], x, sample_c[:,0,:], ':', legend=leg_r)
            helpers.plotXYZ(s_plt[1], x, sample_c[:,1,:], ':', legend=leg_r)

            # Plot theory test
            helpers.plotXYZ(s_plt[0], x, sample_theory[:,0,:], xlabel=xl, ylabel=yl_f, legend=leg_t)
            helpers.plotXYZ(s_plt[1], x, sample_theory[:,1,:], xlabel=xl, ylabel=yl_t, legend=leg_t)

            plt.show()

    if plot_hist:
        return
    else:
        fig, s_plt = plt.subplots(2, 3)

    # Print Histogram
    for si in range(2):
        s_plt[si][0].set_ylabel('Normalized Density')
        s_plt[si][0].set_xlabel('Force X(N)' if si == 0 else 'Torque X(Nm)')
        s_plt[si][1].set_xlabel('Force Y(N)' if si == 0 else 'Torque Y(Nm)')
        s_plt[si][2].set_xlabel('Force Z(N)' if si == 0 else 'Torque Z(Nm)')

        for axis in range(3):
            # Obtain and Normalize data
            data = np.sort(sample_diff[:, si, axis])
            # Normalize data
            data_hist = []
            norm_value = 360
            norm_range = int(math.floor(len(data) / norm_value))
            for i in range(norm_value):
                data_hist.append(mean(data[i*norm_range:(i+1)*norm_range]))
            # Add data to histogram
            s_plt[si][axis].yaxis.set_ticks([])
            s_plt[si][axis].hist(data_hist, 
                bins=20,
                histtype='bar',
                color=color[axis], 
                ec='black')

    plt.show()


def testObjectWeight():
    samples = openTests(15)

    weight_tests = np.empty([2, 3, 10, 3])
    weight_tests[0,0] = samples[1][:,:3] # Xpos
    weight_tests[0,1] = samples[3][:,:3] # Ypos
    weight_tests[0,2] = samples[5][:,:3] # Zpos
    weight_tests[1,0] = samples[0][:,:3] # Xneg
    weight_tests[1,1] = samples[2][:,:3] # Yneg
    weight_tests[1,2] = samples[4][:,:3] # Zneg

    fig, s_plt = plt.subplots(2, 3, sharex='col', sharey='row')

    titles=[
        'X Positive',
        'Y Positive',
        'Z Positive',
        'X Negative',
        'Y Negative',
        'Z Negative'
    ]

    weights_raw = []
    weights_dls = []
    weights_final = []

    p_idx = 0
    for x in range(2):
        for y in range(3):
            sample_force = weight_tests[x, y, :, :]
            weight = np.linalg.norm(sample_force, axis=1)
            print('')
            print(titles[p_idx])
            weights_raw += list(weight/9.81)
            print('Average Weight - %f' % (np.mean(weight)/9.81))
            sample_force[:, 0] /= 0.956
            sample_force[:, 1] /= 1.115
            sample_force[:, 2] *= 1.230
            weight = np.linalg.norm(sample_force, axis=1)
            weights_dls += list(weight/9.81)
            print('DLS Weight - %f' % (np.mean(weight)/9.81))
            sample_force[:,:] /= 1.2
            weight = np.linalg.norm(sample_force, axis=1)
            weights_final += list(weight/9.81)
            print('Correct Weight - %f' % (np.mean(weight)/9.81))
            
            s_plt[x,y].plot(range(10), weight, 'k+', label='weight')
            helpers.plotXYZ(s_plt[x,y], range(10), sample_force, legend=['x', 'y', 'z'])
            s_plt[x,y].set_title(titles[p_idx], loc='center')

            p_idx += 1
    
    print("Average Raw", np.mean(weights_raw))
    print("Std Raw", np.std(weights_raw))
    print("Average DLS", np.mean(weights_dls))
    print("Std DLS", np.std(weights_dls))
    print("Average Correct", np.mean(weights_final))
    print("Std Correct", np.std(weights_final))


    plt.tight_layout()
    plt.show()
    plt.cla()


def main():
    rospy.init_node("final", anonymous = False)
    # test0noGripperNoPay()
    # test1to3noGripper(1)
    # test1to3noGripper(2)
    # test1to3noGripper(3)
    # test4gripperCouple()
    # test5gripperNoPay()
    # test8to9object(8)
    # test8to9object(9)
    # test12gripperZNoPay()
    # test14zObject()
    # testTheoryModel()
    testObjectWeight()




if __name__ == "__main__":
    main()

