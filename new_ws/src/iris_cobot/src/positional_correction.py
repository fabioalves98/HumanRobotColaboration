#!/usr/bin/env python
import pickle
import numpy as np
import matplotlib.pyplot as plt
from correct import BASE_DIR

import helpers

def main():
    # Final position of correction file
    correct_file_path = '/curves/wrench_correct_final.list'
    # Using 8 testes without gripper for positional correction
    sample_files_path = '/record/13-07_06/TCWNG'

    correct_aux = np.empty([0, 360, 2, 3])

    x = np.arange(-180,180,1)

    idx = 0
    for w_1 in helpers.ANGLES:
        for w_2 in helpers.ANGLES:
            if (w_1, w_2) not in helpers.FORBIDDEN:
                if w_1 != 0:
                    idx += 1
                    continue

                sample = helpers.openList('%s%d_%d_%d.list' % (sample_files_path, idx, w_1, w_2))
                correct_aux = np.append(correct_aux, [sample], axis=0)

                idx += 1

    correct = np.mean(correct_aux, axis=0)

    with open(BASE_DIR + correct_file_path, 'w') as f:
        pickle.dump(correct, f)

    helpers.plotXYZ(plt, x, correct[:,0,:])
    helpers.plotXYZ(plt, x, correct[:,1,:])

    plt.show()


if __name__ == '__main__':
    main()