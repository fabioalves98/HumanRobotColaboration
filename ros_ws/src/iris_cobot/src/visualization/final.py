#!/usr/bin/env python
import os
import numpy as np
import matplotlib.pyplot as plt

import cobot.helpers as helpers

W_1 = -90
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
    '/record/last/15_z_gripper_object_1Kg/'
]

correct_w3 = '/curves/wrench_correct_final_720.list'

def open5Tests(test_idx):
    samples = []

    # Open all testes inside specific test folder
    for test_file in os.listdir(helpers.BASE_DIR + tests[test_idx]):
        try:
            sample = helpers.openList(tests[test_idx] + test_file)
            
            # Remove initial outliers
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
    samples = open5Tests(0)
    average = np.empty((5, 720, 2, 3))
    
    fig, s_plt = plt.subplots(2)
    x = np.arange(-360, 360, 1)

    for i in range(5):
        average[i] = samples[i]
        helpers.plotXYZ(s_plt[0], x, samples[i][:,0,:], ':', 0.5, 'Force')
        helpers.plotXYZ(s_plt[1], x, samples[i][:,1,:], ':', 0.5, 'Torque')

    average = np.mean(average, axis=0)

    helpers.plotXYZ(s_plt[0], x, average[:,0,:])
    helpers.plotXYZ(s_plt[1], x, average[:,1,:])

    plt.show()


def test1to3noGripper(test_idx):
    samples = open5Tests(test_idx)

    fig, s_plt = plt.subplots(2)
    x = np.arange(-360, 360, 1)

    # Plot original samples
    for sample in samples:
        helpers.plotXYZ(s_plt[0], x, sample[:,0,:], ':', 0.3)
        helpers.plotXYZ(s_plt[1], x, sample[:,1,:], ':', 0.3)

    # Plot correction curve
    correction = helpers.openList(correct_w3)
    helpers.plotXYZ(s_plt[0], x, correction[:,0,:], '--')
    helpers.plotXYZ(s_plt[1], x, correction[:,1,:], '--')

    # Plot corrected samples
    for sample in samples:
        corrected_sample = sample - correction
        helpers.plotXYZ(s_plt[0], x, corrected_sample[:,0,:], ':', title='Force')
        helpers.plotXYZ(s_plt[1], x, corrected_sample[:,1,:], ':', title='Torque')

    plt.show()


def main():
    # test0noGripperNoPay()
    # test1to3noGripper(1)
    # test1to3noGripper(2)
    test1to3noGripper(3)

    


if __name__ == "__main__":
    main()

