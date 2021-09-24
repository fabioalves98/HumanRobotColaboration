#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt

import cobot.helpers as helpers

tests = [
    '/record/last/1_noGripper_0Pay/T',
    '/record/last/2_noGripper_15Pay/TP',
    '/record/last/3_noGripper_15Pay_42Cog/TPC',
    '/record/last/4_noGripper_15Pay_100Cog/TPC',
    '/record/last/5_gripperCoupled_0Pay/TGc',
    '/record/last/6_gripper_0Pay/TG',
    '/record/last/7_gripper_15Pay/TGP',
    '/record/last/8_gripper_15Pay_42Cog/TGPC',
    '/record/last/9_gripper_object_1Kg/TGO',
    '/record/last/10_gripper_object_2Kg/TGO',
    '/record/last/11_z_noGripper_0Pay/TZ',
    '/record/last/12_z_noGripper_15Pay_42Cog/TZPC',
    '/record/last/13_z_gripper_0Pay/TZG',
    '/record/last/14_z_gripper_15Pay_42Cog/TZGPC',
    '/record/last/15_z_gripper_object_1Kg/TZGO'
]

def main():
    x = np.arange(-180, 180, 1)

    idx = 0

    for w_1 in [-90]:
        for w_2 in [-90, -45, 0, 45, 90]:
            try:
                sample = helpers.openList('/%s%d_%d_%d.list' % 
                    (tests[14], idx, w_1, w_2))
            except IOError:
                break

            # Check for outliers
            for i in range(359):
                for e in range(3):
                    if abs(sample[i, 0, e] - sample[i + 1, 0, e]) > 1:
                        sample[i, 0, e] = sample [i+1, 0, e]
                    if abs(sample[i, 1, e] - sample[i + 1, 1, e]) > 0.2:
                        sample[i, 1, e] = sample [i+1, 1, e]

            # Plot samples
            helpers.plotXYZ(plt, x, sample[:,0,:], ':')
            # helpers.plotXYZ(plt, x, sample[:,1,:], ':')

            idx += 1
        
        plt.show()
        plt.cla()



if __name__ == "__main__":
    main()

