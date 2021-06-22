#!/usr/bin/env python
import pickle
import numpy as np
import matplotlib.pyplot as plt

import cobot.helpers as helpers

def correct(sample):
    correct_matrix = helpers.openList('/curves/wrench_correct_final.list')

    return sample - correct_matrix


def wrench_correct():
    x = np.arange(-180,180,1)

    correct_file_path = '/curves/wrench_correct_final.list'
    correct_matrix = helpers.openList(correct_file_path)

    fig, s_plt = plt.subplots(2)

    helpers.plotXYZ(s_plt[0], x, correct_matrix[:,0,:], title='Force' )
    helpers.plotXYZ(s_plt[1], x, correct_matrix[:,1,:], title='Torque' )

    plt.show()


def main():
    # Real ORbot 57 tests file path
    sample_r_files_path = '/record/13-07_06/TCW'
    # Sim Robot 57 tests file path
    sample_t_files_path = '/record/TCWT'

    x = np.arange(-180, 180, 1)

    idx = 0
    for w_1 in helpers.ANGLES:
        for w_2 in helpers.ANGLES:
            if (w_1, w_2) not in helpers.FORBIDDEN:
                
                # Load Samples
                sample_r = helpers.openList('%s%d_%d_%d.list' % (sample_r_files_path, idx, w_1, w_2))
                sample_t = helpers.openList('%s%d_%d_%d.list' % (sample_t_files_path, idx, w_1, w_2))

                # Correct Real Sample
                sample_r = correct(sample_r)
                
                # Offset Theory Sample
                sample_t -= sample_t[180,:,:]

                helpers.plotXYZ(plt, x, sample_r[:,1,:])
                helpers.plotXYZ(plt, x, sample_t[:,1,:], ':')

                plt.show()
                plt.cla()

                idx += 1



if __name__ == "__main__":
    # wrench_correct()
    main()