#!/usr/bin/env python
import pickle
import rosbag
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib as mpl
import matplotlib.pyplot as plt


from cobot.helpers import BASE_DIR, openList, fkService, plotXYZ

JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint'
]

def trajJoints2Poses():
    bag = rosbag.Bag(BASE_DIR + '/curves/traj_obstacle.bag')

    # Read joint values from trajectory bag
    traj_joints = []
    for topic, msg, t in bag.read_messages():
        joints = []
        for j_name in JOINT_NAMES:
            joints.append(msg.position[msg.name.index(j_name)])

        traj_joints.append(joints)
    
    bag.close()

    # Obtain poses from FK
    traj_poses = []
    for joints in traj_joints:
        traj_poses.append(fkService(joints))

    # Save poses to file
    with open(BASE_DIR + '/curves/traj_obstacle_poses.list', 'w') as f:
        pickle.dump(traj_poses, f)

    print("Converted joint trajectory bag into list of poses file")


def plotTrajPoses():
    traj_n_obs = openList('/curves/traj_no_obstacle_poses.list')
    traj_obs = openList('/curves/traj_obstacle_poses.list')

    trajectories = [traj_n_obs, traj_obs]
    mpl.rcParams['font.size'] = 15

    
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.set_xlabel('X(m)')
    ax.set_ylabel('Y(m)')
    ax.set_zlabel('Z(m)')
    ax.xaxis.set_rotate_label(False) 
    ax.yaxis.set_rotate_label(False) 
    ax.zaxis.set_rotate_label(False) 
    ax.set_xlim3d(0, 1.2)
    ax.set_ylim3d(0, 1.2)
    ax.set_zlim3d(0, 1)
    
    
    lines = ["k--", "k"]
    labels = ["Original", "Adjusted"]

    for traj in trajectories:
        idx = trajectories.index(traj)
        traj_len = len(traj)
        x = range(traj_len)

        traj_xyz = np.empty((traj_len, 3))
        for i in x:
            traj_xyz[i][0] = traj[i].translation.x
            traj_xyz[i][1] = traj[i].translation.y
            traj_xyz[i][2] = traj[i].translation.z

        ax.plot(traj_xyz[:,0], traj_xyz[:,1], traj_xyz[:,2], lines[idx], label=labels[idx])
        ax.legend()


    plt.show()


if __name__ == '__main__':
    # trajJoints2Poses()
    plotTrajPoses()