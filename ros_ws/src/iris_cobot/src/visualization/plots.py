#!/usr/bin/env python
import pickle
import numpy as np
import matplotlib.pyplot as plt

import cobot.helpers as helpers
from cobot.hand_guiding.ft_theory import theoryFT


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
    # Real Robot 57 tests file path
    sample_real_files_path = '/record/13-07_06/TCW'
    # Sim Robot 57 tests file path
    sample_t_files_path = '/record/14-15_06/TCWT'

    # Test theory model
    poses = None
    with open(helpers.BASE_DIR + '/record/poses.list') as f:
        poses = np.array(pickle.load(f))

    x = np.arange(-180, 180, 1)

    idx = 0
    for w_1 in helpers.ANGLES:
        for w_2 in helpers.ANGLES:
            if (w_1, w_2) not in helpers.FORBIDDEN:

                print(idx)
                
                # Load Real Samples
                sample_real = helpers.openList('%s%d_%d_%d.list' % (sample_real_files_path, idx, w_1, w_2))
                
                # Create theory sample
                sample_theory = np.empty([360, 2, 3])
                for i in range(0, len(poses[idx])):
                    theory = theoryFT(poses[idx][i].orientation)
                    sample_theory[i][0] = [theory.force.x, theory.force.y, theory.force.z]
                    sample_theory[i][1] = [theory.torque.x, theory.torque.y, theory.torque.z]

                # Correct Real Sample
                sample_real = correct(sample_real)
                
                # Offset Theory Sample
                sample_theory -= sample_theory[180,:,:]

                helpers.plotXYZ(plt, x, sample_real[:,1,:])
                helpers.plotXYZ(plt, x, sample_theory[:,1,:], ':')

                plt.show()
                plt.cla()

                idx += 1


if __name__ == "__main__":
    main()
    # wrench_correct()