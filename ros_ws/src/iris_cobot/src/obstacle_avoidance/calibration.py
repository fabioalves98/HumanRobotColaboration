#!/usr/bin/env python
import os, shutil
import rospy, rospkg
from math import pi
from std_srvs.srv import Empty
from easy_handeye.srv import TakeSample, ComputeCalibration

import cobot.helpers as helpers

DEFAULT_HANDEYE_NAMESPACE = '/easy_handeye_eye_on_base'
CALIBRATION_FILEPATH = '~/.ros/easy_handeye' + DEFAULT_HANDEYE_NAMESPACE + ".yaml"
BASEDIR = rospkg.RosPack().get_path('iris_cobot')


rotations = [
    [pi/9, 0, 0],
    [-pi/9, 0, 0],
    [0, pi/9, 0],
    [0, -pi/9, 0],
    [0, 0, pi/9],
    [0, 0, -pi/9]
]

def save_calibration_to_basedir():
    ''' Copies the saved calibration file in the calibration filepath to the specified basedir.
        This function should only be called if the calibration is already saved in the .ros dir'''
    try:
        src = os.path.expanduser("~") + CALIBRATION_FILEPATH[1:]
        dest = BASEDIR + "/yaml/easy_handeye_eye_on_base.yaml"
        shutil.copyfile(src, dest)
    except Exception as e:
        rospy.logerr(" Error while saving file to '" + dest + "'")
        rospy.logerr(e)


def take_sample():
    rospy.wait_for_service('/easy_handeye_eye_on_base/take_sample', timeout=2.5)
    take_sample_srv = rospy.ServiceProxy('/easy_handeye_eye_on_base/take_sample', TakeSample)
    vals = take_sample_srv()
    print("New sample taken")
    # transforms = vals.samples.camera_marker_samples.transforms
    # print_samples([transforms[len(transforms)-1]])
    return True


def compute_calibration():
    rospy.wait_for_service('/easy_handeye_eye_on_base/compute_calibration', timeout=2.5)
    compute_calibration_srv = rospy.ServiceProxy('/easy_handeye_eye_on_base/compute_calibration', ComputeCalibration)
    print("Computing calibration")
    result = compute_calibration_srv()
    print("Finished calibration.")
    print(result)
    print("Saving calibration to: " + CALIBRATION_FILEPATH + " and " + BASEDIR + "/yaml")
    rospy.wait_for_service('/easy_handeye_eye_on_base/save_calibration', timeout=2.5)
    save_calibration = rospy.ServiceProxy('/easy_handeye_eye_on_base/save_calibration', Empty)
    save_calibration()


def main():
    rospy.init_node('calibration', anonymous=True)

    # Send to default position
    position = 'workspace'
    helpers.samiAliasService(position)

    # Make a set of movements and take samples
    for rotation in rotations:
        for i in range(2):
            helpers.samiMoveService([0, 0, 0].append(rotation))
            take_sample()
        helpers.samiAliasService(position)

    # Compute calibration and save
    compute_calibration()
    save_calibration_to_basedir()


if __name__ == "__main__":
    main()