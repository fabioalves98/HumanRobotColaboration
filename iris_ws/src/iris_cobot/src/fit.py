#!/usr/bin/env python
import rospy, rospkg, pickle

from math import radians
import matplotlib.pyplot as plt
import numpy as np
from scipy import optimize

from helpers import plotXYZ

BASE_DIR = rospkg.RosPack().get_path('iris_cobot')

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
tests_payload_gripper = [
    '/record/4-05_03/TGB_temp.list',
    '/record/4-05_03/TGB_0Kg_temp.list',
    '/record/4-05_03/TGB_3Kg_temp.list',
]
tests_payload = [
    '/record/5-08_03/TP1_0Kg_temp.list',
    '/record/5-08_03/TP1_0.2Kg_temp.list',
    '/record/5-08_03/TP1_1Kg_temp.list',
    '/record/5-08_03/TP2_0Kg_temp.list',
    '/record/5-08_03/TP2_0.2Kg_temp.list',
    '/record/5-08_03/TP2_1Kg_temp.list',
    '/record/5-08_03/TP3_0Kg_temp.list',
    '/record/5-08_03/TP3_0.2Kg_temp.list',
    '/record/5-08_03/TP3_1Kg_temp.list',
]
tests_gripper_coupling = [
    '/record/5-08_03/TPB_0Kg_temp.list',
    '/record/5-08_03/TPBG_0Kg_temp.list',
]
tests_gripper_weight = [
    '/record/5-08_03/TPBG_1.2Kg_temp.list',
    '/record/5-08_03/TPBG_1.3Kg_temp.list',
    '/record/5-08_03/TPBG_1.4Kg_temp.list',
    '/record/5-08_03/TPBG_1.5Kg_temp.list',
    '/record/5-08_03/TPBG_1.6Kg_temp.list',
    '/record/5-08_03/TPBG_1.7Kg_temp.list',
    '/record/5-08_03/TPBG_1.8Kg_temp.list',
]
test_gripper_weight = '/record/5-08_03/TP3G_1.5Kg_temp.list'

correct_fit = '/curves/wrench_correct_fit.list'
correct_mean = '/curves/wrench_correct_mean.list'
    

def compareCurves(plt, curves):
    x = np.arange(-180,180,1)

    for i in range(len(curves)):
        print(curves[i])
        stream = []
        with open(BASE_DIR + curves[i]) as f:
            stream = pickle.load(f)

        plotXYZ(plt, x, stream, ['',':',':'][i])


def repeatabilityTest(plt):
    x = np.arange(-180,180,1)
    streams = []

    for test in tests_1 + tests_2:
        stream = []
        with open(BASE_DIR + test) as f:
            stream = pickle.load(f)
            streams.append(np.array(stream))

        plotXYZ(plt, x, stream, ':')

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

    plotXYZ(plt, x, zero_0)

    zero_75 = []
    with open(BASE_DIR + test_75) as f:
        zero_75 = pickle.load(f)

    plotXYZ(plt, x, zero_75, '--')
    
    comp_value = zero_0[len(zero_0)/2 - 75]
    zero_75_comp = []
    for i in range(len(zero_75)):
        zero_75_comp.append((zero_75[i][0] + comp_value[0], zero_75[i][1] + comp_value[1], zero_75[i][2] + comp_value[2]))

    plotXYZ(plt, x, zero_75_comp, ':')


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

        plotXYZ(sub_plots[i], x, stream, ':')


def positionalDriftTest(plt):
    test_no_move = '/record/3-02_03/T1_P1_temp.list'
    test_move_wo_reset = '/record/3-02_03/TP_P1_wo_reset_temp.list'
    test_move_w_reset = '/record/3-02_03/TP_P1_w_reset_temp.list'

    x = np.arange(-180,180,1)
    
    no_move = []
    with open(BASE_DIR + test_no_move) as f:
        no_move = pickle.load(f)

    plotXYZ(plt, x, no_move, ':')

    move_wo_reset = []
    with open(BASE_DIR + test_move_wo_reset) as f:
        move_wo_reset = pickle.load(f)

    plotXYZ(plt, x, move_wo_reset, '')

    move_w_reset = []
    with open(BASE_DIR + test_move_w_reset) as f:
        move_w_reset = pickle.load(f)
    
    plotXYZ(plt, x, move_w_reset, '--')


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


def correct(plt, curve, alpha=0.5):
    x = np.arange(-180,180,1)

    correct = []
    with open(BASE_DIR + correct_mean) as f:
        correct = pickle.load(f)

    if plt:
        plotXYZ(plt, x, np.transpose(correct), ':', alpha=alpha)
    
    to_correct = []
    with open(BASE_DIR + curve) as f:
        to_correct = pickle.load(f)

    if plt:
        plotXYZ(plt, x, to_correct, ':', alpha=alpha)

    difference = np.array(to_correct) - np.array(correct).transpose()
    
    if plt:
        plotXYZ(plt, x, difference)

    return difference


def payloadTest(plt, pos):
    x = np.arange(-180,180,1)
    default = []
    with open(BASE_DIR + tests_payload[pos * 3 + 0]) as f:
        default = pickle.load(f)

    plotXYZ(plt, x, default)

    bit_heavier = []
    with open(BASE_DIR + tests_payload[pos * 3 + 1]) as f:
        bit_heavier = pickle.load(f)

    plotXYZ(plt, x, bit_heavier, '--')

    heavier = []
    with open(BASE_DIR + tests_payload[pos * 3 + 2]) as f:
        heavier = pickle.load(f)

    plotXYZ(plt, x, heavier, '+')


def gripperCoupling(plt):
    x = np.arange(-180,180,1)

    default = []
    with open(BASE_DIR + tests_gripper_coupling[0]) as f:
        default = pickle.load(f)

    plotXYZ(plt, x, default)

    gripper = []
    with open(BASE_DIR + tests_gripper_coupling[1]) as f:
        gripper = pickle.load(f)

    plotXYZ(plt, x, gripper, '+')

def getWeight(plt, curve):
    x = np.arange(-180,180,1)

    corrected = correct(None, curve)

    weight = np.empty(360)
    z_comp = np.mean(corrected[:, 2])
    print('Z comp - ', z_comp)
    
    weight = np.sqrt( np.power(corrected[:,0], 2) + np.power(corrected[:,1], 2) +
                      np.power(corrected[:,2] - z_comp, 2))
    print('Mean Weight - ', np.mean(weight))

    plotXYZ(plt, x, corrected)
    plt.plot(x, weight, 'k')


def gripperWeightTest(plt):
    x = np.arange(-180,180,1)

    default = []
    with open(BASE_DIR + tests_payload[9]) as f:
        default = pickle.load(f)

    plotXYZ(plt, x, default)

    for i in range(11, len(tests_payload)):
        stream = []
        with open(BASE_DIR + tests_payload[i]) as f:
            stream = pickle.load(f)

        plotXYZ(plt, x, stream, ':')

def main():
    rospy.init_node("fit", anonymous = False)
    
    # plt.ylim([-10, 15.0])

    # Quickly compare curves
    # compareCurves(plt, [tests_payload[6], tests_payload[14], tests_payload[18]])

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
    # correct(plt, tests_gripper[2])

    # Test different payloads without gripper
    # pos = 2
    # payloadTest(plt, pos)
    # correct(plt, tests_payload[pos * 3 + 2])

    # Test coupling gripper without reset
    # gripperCoupling(plt)
    # correct(plt, tests_gripper_coupling[1])
    getWeight(plt, tests_gripper_coupling[1])

    # Test different payload with gripper to get best match
    # gripperWeightTest(plt)

    plt.show()

if __name__ == "__main__":
    main()