#!/usr/bin/env python
import os, pickle
import numpy as np

import cobot.helpers as helpers

def main():
    # Final position of correction file
    correct_file_path = '/curves/wrench_correct_final_720.list'
    # Using 5 tests without gripper for positional correction
    test_directory = '/record/last/1_noGripper_0Pay/'

    correct_aux = np.empty([0, 720, 2, 3])

    for test_file in os.listdir(helpers.BASE_DIR + test_directory):
        sample = helpers.openList(test_directory + test_file)
        
        # Remove initial outliers
        for e in range(3):
            sample[0, 0, e] = sample[2, 0, e]
            sample[1, 0, e] = sample[2, 0, e]
            sample[0, 1, e] = sample[2, 1, e]
            sample[1, 1, e] = sample[2, 1, e]
            
        correct_aux = np.append(correct_aux, [sample], axis=0)

    correct = np.mean(correct_aux, axis=0)

    with open(helpers.BASE_DIR + correct_file_path, 'w') as f:
        pickle.dump(correct, f)


if __name__ == '__main__':
    main()