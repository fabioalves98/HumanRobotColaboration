#!/usr/bin/env python
import rospy, rospkg, pickle

from math import radians
import matplotlib.pyplot as plt
import numpy as np
from scipy import optimize

BASE_DIR = rospkg.RosPack().get_path('ur10e_control')

tests_1 = [
    '/record/1-13_01/T1_temp.list',
    '/record/1-13_01/T2_temp.list',
    '/record/1-13_01/T3_temp.list',
    '/record/1-13_01/T4_temp.list',
    '/record/1-13_01/T5_temp.list'
]
tests_2 = [
    '/record/2-01_03/T1_temp.list',
    '/record/2-01_03/T2_temp.list',
    '/record/2-01_03/T3_temp.list',
    '/record/2-01_03/T4_temp.list',
    '/record/2-01_03/T5_temp.list'
]
tests_gripper = [
    '/record/4-05_03/TG1_temp.list',
    '/record/4-05_03/TG2_temp.list',
    '/record/4-05_03/TG3_temp.list',
    '/record/4-05_03/TG4_temp.list',
    '/record/4-05_03/TG5_temp.list'
]
correct_fit = '/curves/wrench_correct_fit.list'
correct_mean = '/curves/wrench_correct_mean.list'
    

def repeatabilityTest(plt):
    x = np.arange(-180,180,1)
    streams = []

    for test in tests_1 + tests_2:
        stream = []
        with open(BASE_DIR + test) as f:
            stream = pickle.load(f)
            streams.append(np.array(stream))

        plt.plot(x, [t[0] for t in stream], 'r:')
        plt.plot(x, [t[1] for t in stream], 'g:')
        plt.plot(x, [t[2] for t in stream], 'b:')

    # Mean Min Max differences
    diff = []
    for i in range(len(streams)):
        for j in range(i, len(streams)):
            if i == j: 
                continue
            diff.append( np.abs(streams[i] - streams[j]))

    diffs = np.array(diff)
    print(diffs.shape)
    tests_mean = np.mean(diff, axis = 0)
    print(tests_mean.shape)
    axis_min = tests_mean.min(axis = 0)
    axis_max = tests_mean.max(axis = 0)
    axis_mean = np.mean(tests_mean, axis = 0)
    axis_std = np.std(tests_mean, axis = 0)
    
    print('Min - ', axis_min)
    print('Max - ', axis_max)
    print('Mean - ', axis_mean)
    print('Std Dev - ', axis_std)


def resetTest(plt):
    test_0 = '/record/2-01_03/T5_temp.list'
    test_75 = '/record/2-01_03/T6_temp.list'
    
    x = np.arange(-180,180,1)
    
    zero_0 = []
    with open(BASE_DIR + test_0) as f:
        zero_0 = pickle.load(f)

    plt.plot(x, [t1[0] for t1 in zero_0], 'r')
    plt.plot(x, [t1[1] for t1 in zero_0], 'g')
    plt.plot(x, [t1[2] for t1 in zero_0], 'b')

    zero_75 = []
    with open(BASE_DIR + test_75) as f:
        zero_75 = pickle.load(f)

    plt.plot(x, [t2[0] for t2 in zero_75], 'r--')
    plt.plot(x, [t2[1] for t2 in zero_75], 'g--')
    plt.plot(x, [t2[2] for t2 in zero_75], 'b--')
    
    comp_value = zero_0[len(zero_0)/2 - 75]
    zero_75_comp = []
    for i in range(len(zero_75)):
        zero_75_comp.append((zero_75[i][0] + comp_value[0], zero_75[i][1] + comp_value[1], zero_75[i][2] + comp_value[2]))

    plt.plot(x, [t3[0] for t3 in zero_75_comp], 'r:')
    plt.plot(x, [t3[1] for t3 in zero_75_comp], 'g:')
    plt.plot(x, [t3[2] for t3 in zero_75_comp], 'b:')


def temporalDriftTest(plt):
    tests = [
        '/record/2-01_03/TD_P2_temp.list',
        '/record/2-01_03/TD_P3_temp.list',
        '/record/2-01_03/TD_P4_temp.list'
    ]

    x = np.arange(0, 3000,1)
    fig, sub_plots = plt.subplots(3)

    for plt in sub_plots:
        plt.set(ylim=(-1, 2))

    for i in range(len(tests)):
        stream = []
        with open(BASE_DIR + tests[i]) as f:
            stream = pickle.load(f)

        sub_plots[i].plot(x, [t[0] for t in stream], 'r:')
        sub_plots[i].plot(x, [t[1] for t in stream], 'g:')
        sub_plots[i].plot(x, [t[2] for t in stream], 'b:')


def positionalDriftTest(plt):
    test_no_move = '/record/3-02_03/T1_P1_temp.list'
    test_move_wo_reset = '/record/3-02_03/TP_P1_wo_reset_temp.list'
    test_move_w_reset = '/record/3-02_03/TP_P1_w_reset_temp.list'

    x = np.arange(-180,180,1)
    
    no_move = []
    with open(BASE_DIR + test_no_move) as f:
        no_move = pickle.load(f)

    plt.plot(x, [t1[0] for t1 in no_move], 'r:')
    plt.plot(x, [t1[1] for t1 in no_move], 'g:')
    plt.plot(x, [t1[2] for t1 in no_move], 'b:')

    move_wo_reset = []
    with open(BASE_DIR + test_move_wo_reset) as f:
        move_wo_reset = pickle.load(f)

    plt.plot(x, [t1[0] for t1 in move_wo_reset], 'r')
    plt.plot(x, [t1[1] for t1 in move_wo_reset], 'g')
    plt.plot(x, [t1[2] for t1 in move_wo_reset], 'b')

    move_w_reset = []
    with open(BASE_DIR + test_move_w_reset) as f:
        move_w_reset = pickle.load(f)

    plt.plot(x, [t1[0] for t1 in move_w_reset], 'r--')
    plt.plot(x, [t1[1] for t1 in move_w_reset], 'g--')
    plt.plot(x, [t1[2] for t1 in move_w_reset], 'b--')


def fitFunction(plt):
    x_global = np.arange(-180,180,1)

    streams = []
    for test in tests_1 + tests_2:
        stream = []
        with open(BASE_DIR + test) as f:
            stream = pickle.load(f)
            streams.append(np.array(stream))

    data = np.array(streams)
    colors = ['r', 'g', 'b']
    init_params = [
        [1,   2, -60,  2],
        [1.5, 2,  40, -0.5],
        [2,   2,  40, -2.5]
    ]

    correction_fit = np.empty((3, 360))
    correction_mean = np.empty((3, 360))

    def sin_func(x, a, b, c, d):
        return a * np.sin(np.radians(b * (x + c))) + d

    for i in range(3):
        axis = data[:,:,i]
        axis_mean = np.mean(axis, axis=0)

        for test in axis:
            plt.plot(x_global, test, colors[i] + ':', alpha=0.5)

        plt.plot(x_global, axis_mean, colors[i])

        prm, _ = optimize.curve_fit(sin_func, x_global, axis_mean, p0=init_params[i])
        fit = prm[0] * np.sin(np.radians(prm[1]*(x_global + prm[2]))) + prm[3]
        correction_fit[i] = fit
        correction_mean[i] = axis_mean
        plt.plot(x_global, fit, colors[i])

    with open(BASE_DIR + correct_fit, 'w') as f:
        pickle.dump(correction_fit, f)

    with open(BASE_DIR + correct_mean, 'w') as f:
        pickle.dump(correction_mean, f)


def gripperDiference(plt):
    x = np.arange(-180,180,1)

    correct = []
    with open(BASE_DIR + correct_mean) as f:
        correct = pickle.load(f)

    plt.plot(x, [t1 for t1 in correct[0]], 'r:')
    plt.plot(x, [t1 for t1 in correct[1]], 'g:')
    plt.plot(x, [t1 for t1 in correct[2]], 'b:')
    
    w_gripper = []
    with open(BASE_DIR + tests_gripper[0]) as f:
        w_gripper = pickle.load(f)

    plt.plot(x, [t2[0] for t2 in w_gripper], 'r:')
    plt.plot(x, [t2[1] for t2 in w_gripper], 'g:')
    plt.plot(x, [t2[2] for t2 in w_gripper], 'b:')

    difference = np.array(w_gripper).transpose() - np.array(correct)
        
    plt.plot(x, [t3 for t3 in difference[0]], 'r')
    plt.plot(x, [t3 for t3 in difference[1]], 'g')
    plt.plot(x, [t3 for t3 in difference[2]], 'b')


def main():
    rospy.init_node("fit", anonymous = False)
    
    # plt.ylim([-10, 15.0])

    # Repeatability and variation test
    # repeatabilityTest(plt)

    # Reset in different angles test function
    # resetTest(plt)

    # Temporal drift test
    # temporalDriftTest(plt)

    # Positional drift test
    # positionalDriftTest(plt)

    # Attempt of a sin function to fit results
    # fitFunction(plt)

    # Merge das curvas com e sem gripper
    gripperDiference(plt)

    plt.show()

if __name__ == "__main__":
    main()