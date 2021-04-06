#!/usr/bin/env python
import rospy, rospkg, pickle
import matplotlib.pyplot as plt
import numpy as np
from scipy import optimize

from helpers import openList, plotXYZ

BASE_DIR = rospkg.RosPack().get_path('iris_cobot')

# 1 - 13/01
# Test robot FT sensor in along wrist_3 rotation in 5 different positions
tests_1 = [
    '/record/1-13_01/T1_temp.list',
    '/record/1-13_01/T2_temp.list',
    '/record/1-13_01/T3_temp.list',
    '/record/1-13_01/T4_temp.list',
    '/record/1-13_01/T5_temp.list'
]
# 2 - 01/03
# Verify repeatability of the previous tests
tests_2 = [
    '/record/2-01_03/T1_temp.list',
    '/record/2-01_03/T2_temp.list',
    '/record/2-01_03/T3_temp.list',
    '/record/2-01_03/T4_temp.list',
    '/record/2-01_03/T5_temp.list'
]
# 4 - 05/03
# Perform the same tests with gripper attached (payload - 1.77kg)
tests_gripper = [
    '/record/4-05_03/TG1_temp.list',
    '/record/4-05_03/TG2_temp.list',
    '/record/4-05_03/TG3_temp.list',
    '/record/4-05_03/TG4_temp.list',
    '/record/4-05_03/TG5_temp.list'
]
# 5 - 08/03
# Perform in depth payload test without gripper in 3 different postions (0 | 0.2 | 1kg)
tests_payload = [
    '/record/5-08_03/TP1_0Kg_temp.list',
    '/record/5-08_03/TP1_0.2Kg_temp.list',
    '/record/5-08_03/TP1_1Kg_temp.list',
    '/record/5-08_03/TP2_0Kg_temp.list',
    '/record/5-08_03/TP2_0.2Kg_temp.list',
    '/record/5-08_03/TP2_1Kg_temp.list',
    '/record/5-08_03/TP3_0Kg_temp.list',
    '/record/5-08_03/TP3_0.2Kg_temp.list',
    '/record/5-08_03/TP3_1Kg_temp.list'
]
# Perform gripper coupling tests in a specific position
tests_gripper_coupling = [
    '/record/5-08_03/TPB_0Kg_temp.list',
    '/record/5-08_03/TPBG_0Kg_temp.list',
]
# Perform various tests with gripper changing the programmed payload
tests_gripper_weight = [
    '/record/5-08_03/TPBG_1.2Kg_temp.list',
    '/record/5-08_03/TPBG_1.3Kg_temp.list',
    '/record/5-08_03/TPBG_1.4Kg_temp.list',
    '/record/5-08_03/TPBG_1.5Kg_temp.list',
    '/record/5-08_03/TPBG_1.6Kg_temp.list',
    '/record/5-08_03/TPBG_1.7Kg_temp.list',
    '/record/5-08_03/TPBG_1.8Kg_temp.list'
]
# Sanity check test where the same payload is tested in a different position
test_gripper_weight = '/record/5-08_03/TP3G_1.5Kg_temp.list'
# 6 - 09/03
# Perform in depth center of gravity test with gripper attached
tests_cog = [
    '/record/6-09_03/T1_COG_temp.list',
    '/record/6-09_03/T1_COG_100_temp.list',
    '/record/6-09_03/T1_COG_200_temp.list',
    '/record/6-09_03/T2_COG_temp.list',
    '/record/6-09_03/T2_COG_100_temp.list',
    '/record/6-09_03/T2_COG_200_temp.list',
    '/record/6-09_03/T3_COG_temp.list',
    '/record/6-09_03/T3_COG_100_temp.list',
    '/record/6-09_03/T3_COG_200_temp.list'
]
# Perform in depth gripper tests with different payloads (0 - 3Kg)
tests_payload_gripper = [
    '/record/6-09_03/TG1_0Kg_temp.list',
    '/record/6-09_03/TG1_1.5Kg_temp.list',
    '/record/6-09_03/TG1_3Kg_temp.list',
    '/record/6-09_03/TG2_0Kg_temp.list',
    '/record/6-09_03/TG2_1.5Kg_temp.list',
    '/record/6-09_03/TG2_3Kg_temp.list',
    '/record/6-09_03/TG3_0Kg_temp.list',
    '/record/6-09_03/TG3_1.5Kg_temp.list',
    '/record/6-09_03/TG3_3Kg_temp.list',
    '/record/6-09_03/TG4_0Kg_temp.list',
    '/record/6-09_03/TG4_1.5Kg_temp.list',
    '/record/6-09_03/TG4_3Kg_temp.list',
    '/record/6-09_03/TG5_0Kg_temp.list',
    '/record/6-09_03/TG5_1.5Kg_temp.list',
    '/record/6-09_03/TG5_3Kg_temp.list'
]
# 7 - 15/03
# Perform in depth gripper tests with minor payload diferences and multiple positions
tests_payload_gripper_minor = [
    '/record/7-15_03/TG1_1.4Kg_temp.list',
    '/record/7-15_03/TG1_1.5Kg_temp.list',
    '/record/7-15_03/TG1_1.6Kg_temp.list',
    '/record/7-15_03/TG2_1.4Kg_temp.list',
    '/record/7-15_03/TG2_1.5Kg_temp.list',
    '/record/7-15_03/TG2_1.6Kg_temp.list',
    '/record/7-15_03/TG3_1.4Kg_temp.list',
    '/record/7-15_03/TG3_1.5Kg_temp.list',
    '/record/7-15_03/TG3_1.6Kg_temp.list',
    '/record/7-15_03/TG4_1.4Kg_temp.list',
    '/record/7-15_03/TG4_1.5Kg_temp.list',
    '/record/7-15_03/TG4_1.6Kg_temp.list',
    '/record/7-15_03/TG5_1.4Kg_temp.list',
    '/record/7-15_03/TG5_1.5Kg_temp.list',
    '/record/7-15_03/TG5_1.6Kg_temp.list'
]
# 8 - 17/03
# Create theoretical model for force sensor behavior
test_theory_sensor = '/record/8-17_03/TG3_theory_temp.list'

# 9 - 24/03
# Test robot FT sensor without gripper along wrist_3 in 56 diferent positions
tests_56_no_gripper = '/record/9-24_03/TC'
tests_56_gripper = '/record/9-24_03/TCG'
tests_56_no_payload = '/record/TCGP'

# 10 - 26/03
# Test the relativity of the 56 positions
test_56_relative = '/record/10-26_03/TCG_relative_temp.list'

# Correction curves
correct_fit = '/curves/wrench_correct_fit.list'
correct_mean = '/curves/wrench_correct_mean.list'
correct_56 = '/curves/wrench_correct_56.list'


def compareCurves(plt, curves):
    x = np.arange(-180,180,1)

    for i in range(len(curves)):
        print(curves[i])
        stream = []
        with open(BASE_DIR + curves[i]) as f:
            stream = pickle.load(f)

        stream = np.array(stream)
        if stream.shape == (3, 360):
            stream = stream.transpose()

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
    
    to_correct = curve
    if isinstance(to_correct, str):
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

    fig, sub_plots = plt.subplots(3, 3)

    default = []
    with open(BASE_DIR + tests_gripper_coupling[0]) as f:
        default = pickle.load(f)

    plotXYZ(sub_plots[0, 0], x, default)

    for i in range(len(tests_gripper_weight)):
        test = []
        print('\n%d - %s' % (i, tests_gripper_weight[i]))
        with open(BASE_DIR + tests_gripper_weight[i]) as f:
            test = pickle.load(f)
        
        # Plot gripper
        legend = tests_gripper_weight[i].split('/')[3].split('_')[1]
        pos = i + 1
        plotXYZ(sub_plots[pos/3, pos%3], x, test, title=legend)
        plotXYZ(sub_plots[pos/3, pos%3], x, default, ':')

        # Calculate curve that has the minimum amount of differences
        diff = np.array(default) - np.array(test)
        axes = ['X', 'Y', 'Z']
        print('A - %f  %f' % (np.mean(diff), np.std(diff)))
        for i in range(3):
            print('%s - %f - %f' % (axes[i], np.mean(diff[:,i]), np.std(diff[:,i])))

        # Plot the corrected curve
        correct = np.array(test) - np.array(default)
        plotXYZ(sub_plots[pos/3, pos%3], x, correct, '+')
        sub_plots[pos/3, pos%3].plot(x, np.mean(correct, axis=1), 'k')


def centerOfGravityTest(plt):
    x = np.arange(-180,180,1)

    fig, sub_plots = plt.subplots(3)

    for i in range(3):

        default = []
        with open(BASE_DIR + tests_cog[i]) as f:
            default = pickle.load(f)

        plotXYZ(sub_plots[i], x, default, ':')

        cog_100 = []
        with open(BASE_DIR + tests_cog[i+1]) as f:
            cog_100 = pickle.load(f)

        plotXYZ(sub_plots[i], x, cog_100, '--')

        cog_200 = []
        with open(BASE_DIR + tests_cog[i+2]) as f:
            cog_200 = pickle.load(f)

        plotXYZ(sub_plots[i], x, cog_200, '+')


def gripperPayloadTest(plt, pos):
    x = np.arange(-180,180,1)
    
    lighter = []
    with open(BASE_DIR + tests_payload_gripper[pos * 3 + 0]) as f:
        lighter = pickle.load(f)

    lighter = correct(None, lighter)
    plotXYZ(plt, x, lighter, '--')

    default = []
    with open(BASE_DIR + tests_payload_gripper[pos * 3 + 1]) as f:
        default = pickle.load(f)

    default = correct(None, default)
    plotXYZ(plt, x, default)

    heavier = []
    with open(BASE_DIR + tests_payload_gripper[pos * 3 + 2]) as f:
        heavier = pickle.load(f)

    heavier = correct(None, heavier)
    plotXYZ(plt, x, heavier, '+')


def gripperPayloadMinorTest(plt):
    x = np.arange(-180,180,1)

    # Compare tests_payload_gripper 1.5 with tests_payload_gripper_minor 1.5
    # for i in range(5):
    #     test_before = openList(tests_payload_gripper[i*3+1])
    #     plotXYZ(plt, x, test_before, [':', '', '+', '', ':'][i], 0.5)

    #     test_after = openList(tests_payload_gripper_minor[i*3+1])
    #     plotXYZ(plt, x, test_after, [':', '', '+', '', ':'][i])

    # Compare tests_payload_gripper 1.4 1.5 1.6
    for i in range(5):
        test_1_4 = openList(tests_payload_gripper_minor[i*3])
        plotXYZ(plt, x, test_1_4, '--')

        test_1_5 = openList(tests_payload_gripper_minor[i*3 + 1])
        plotXYZ(plt, x, test_1_5)

        test_1_6 = openList(tests_payload_gripper_minor[i*3 + 2])
        plotXYZ(plt, x, test_1_6, '+')

    #     # print("\nShowing %s" % tests_payload_gripper_minor[i])
    #     # plt.show()
    #     # plt.cla()
    

def gripperTheoreticalTest(plt):
    x = np.arange(-180,180,1)

    # Corrected gripper curve
    plotXYZ(plt, x, correct(None, openList(tests_gripper_coupling[1])), ':')


    # Curve made with theoretical model of sensor behavior
    plotXYZ(plt, x, openList(test_theory_sensor))

    theory = openList(test_theory_sensor)

    weight = np.empty(360)
    weight = np.sqrt( np.power(theory[:,0], 2) + np.power(theory[:,1], 2) +
                      np.power(theory[:,2], 2))
    print('Mean Weight - ', np.mean(weight))
    plt.plot(x, weight, 'k')


def gripperCorrectTest(plt):
    x = np.arange(-180,180,1)

    angles = [-180, -135, -90, -45, 0, 45, 90, 135]
    forbidden = [(45, -180),  (45, -135),
                (90, -180),  (90, -135), (90, 135),
                (135, -180), (135, 135)]

    colors = ['blue' ,'cyan', 'green', 'yellow', 'red', 'pink', 'white', 'black']
    color = ''

    idx=0

    curves = []

    # Observe curves with the same W_1 value - Same color
    for w_1 in angles:
        for w_2 in angles:
            if (w_1, w_2) not in forbidden:
                # Specific test based on W_1
                if color:
                    color_i = colors.index(color)
                    if angles.index(w_1) != color_i:
                        idx += 1
                        continue

                # Specific test based on W_2
                # if w_2 != 0:
                #     idx +=1
                #     continue

                try:
                    # test_grip = openList('%s%d_%d_%d_temp.list' % (tests_56_gripper, idx, w_1, w_2))
                    test_no_payl = openList('%s%d_%d_%d_temp.list' % (tests_56_no_payload, idx, w_1, w_2))
                    # test_no_grip = openList('%s%d_%d_%d_temp.list' % (tests_56_no_gripper, idx, w_1, w_2))
                    # test_grip_correct = test_grip - test_no_grip
                    # curves.append(test_grip)

                    print('Opened - %d - %d - %d' % (idx, w_1, w_2))
                    # plotXYZ(plt, x, test_grip, ':', alpha=0.3)
                    # plotXYZ(plt, x, test_no_grip, '--', alpha=0.3)
                    plotXYZ(plt, x, test_no_payl)
                    # plotXYZ(plt, x, test_grip_correct)

                    idx += 1
                    plt.show()
                    plt.cla()
                except IOError:
                    print('Test does not exist')
                    break

    # Create correction mean
    # correction = np.empty((3, 360))
    # data = np.array(curves)

    # for i in range(3):
    #     axis = data[:,:,i]
    #     axis_mean = np.mean(axis, axis=0) 
    #     correction[i] = axis_mean
    
    # plotXYZ(plt, x, correction)

    # with open(BASE_DIR + correct_56, 'w') as f:
    #     pickle.dump(correction, f)


def gripperRelativeTest(plt):
    x = range(57)

    test_relative = openList(6)

    plotXYZ(plt, x, test_relative)


def main():
    rospy.init_node("fit", anonymous = False)
    
    # plt.ylim([-10, 15.0])

    # Quickly compare curves
    # compareCurves(plt, [test])

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
    # getWeight(plt, tests_gripper_coupling[1])

    # Test different payload values with gripper to get best match
    # gripperWeightTest(plt)

    # Test different center of gravity values with gripper
    # centerOfGravityTest(plt)
    
    # Test different positions and payloads with gripper
    # pos = 4
    # gripperPayloadTest(plt, pos)

    # Test more positions and minor payload changes with gripper
    # gripperPayloadMinorTest(plt)

    # Showcase theoretical model for sensor behavior
    # gripperTheoreticalTest(plt)

    # Test 56 positions in order to correct FT sensor
    gripperCorrectTest(plt)

    # Test the relativity of the 56 positions
    # gripperRelativeTest(plt)

    # plt.show()

if __name__ == "__main__":
    main()