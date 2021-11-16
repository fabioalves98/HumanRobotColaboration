#!/usr/bin/env python
import pickle, os, yaml
import rospy, rosbag
import numpy as np
import tf2_ros
from mpl_toolkits.mplot3d import Axes3D
from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion, PointStamped
import matplotlib as mpl
import matplotlib.pyplot as plt


from cobot.helpers import BASE_DIR, openList, fkService, plotXYZ
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point

JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint'
]

FOLDER = '/record/last/17_trajectory/'

def trajGaz2List():
    bag = rosbag.Bag(BASE_DIR + FOLDER + 'traj_obstacle.bag')

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
    with open(BASE_DIR + FOLDER + 'traj_obstacle_poses.list', 'w') as f:
        pickle.dump(traj_poses, f)

    print("Converted joint trajectory bag into list of poses file")


def trajReal2List():
    for bag_file in sorted(os.listdir(BASE_DIR + FOLDER)):
        if "traj_real" in bag_file:
            bag = rosbag.Bag(BASE_DIR + FOLDER + bag_file)
            print("\n\n")
            print(bag_file)
            print(bag.get_type_and_topic_info())

            traj_joints = []
            for topic, msg, t in bag.read_messages():
                if topic == "/joint_states":
                    if len(msg.name) == 6:
                        joints = []
                        for j_name in JOINT_NAMES:
                            joints.append(msg.position[msg.name.index(j_name)])

                        traj_joints.append(joints)

            bag.close()

            # Obtain poses from FK
            traj_poses = []
            for joints in traj_joints:
                traj_poses.append(fkService(joints))

            with open(BASE_DIR + FOLDER + bag_file[:-4] + '.list', 'w') as f:
                pickle.dump(traj_poses, f)
    

def plotGazTraj():
    traj_n_obs = openList(FOLDER + 'traj_no_obstacle_poses.list')
    traj_obs = openList(FOLDER + 'traj_obstacle_poses.list')

    trajectories = [traj_n_obs, traj_obs]

    mpl.rc('font', size=15)
    mpl.rc('axes', labelsize=20)
    
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.set_xlabel('\n\nX(m)')
    ax.set_ylabel('\n\nY(m)')
    ax.set_zlabel('Z(m)     ')
    ax.xaxis.set_rotate_label(False) 
    ax.yaxis.set_rotate_label(False) 
    ax.zaxis.set_rotate_label(False) 
    ax.set_xlim3d(0, 1)
    ax.set_ylim3d(0, 1)
    ax.set_zlim3d(0, 1)    
    lines = ["k--", "b"]
    labels = ["Original", "Adjusted"]

    # Plot Trajectories
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

    # Plot Obstacle
    obs_u = np.linspace(0, 2 * np.pi, 100)
    obs_v = np.linspace(0, np.pi, 100)
    obs_x = 0.8 + 0.1 * np.outer(np.cos(obs_u), np.sin(obs_v))
    obs_y = 0.8 + 0.1 * np.outer(np.sin(obs_u), np.sin(obs_v))
    obs_z = 0.35 + 0.1 * np.outer(np.ones(np.size(obs_u)), np.cos(obs_v))
    
    ax.plot_wireframe(obs_x, obs_y, obs_z, color='r', rcount = 20, ccount = 20)

    plt.show()


def plotRealTraj():
    # Get trajectories lists
    traj_n_obs = openList(FOLDER + 'traj_real_no_obstacle.list')
    traj_obs_10 = openList(FOLDER + 'traj_real_10.list')
    traj_obs_30 = openList(FOLDER + 'traj_real_30.list')
    traj_obs_50 = openList(FOLDER + 'traj_real_50.list')
    traj_obs_60 = openList(FOLDER + 'traj_real_60.list')
    traj_obs_70 = openList(FOLDER + 'traj_real_70.list')
    traj_obs_80 = openList(FOLDER + 'traj_real_80.list')

    trajectories = [traj_n_obs, traj_obs_10, traj_obs_30, traj_obs_50, traj_obs_60, traj_obs_70, traj_obs_80]

    mpl.rc('font', size=15)
    mpl.rc('axes', labelsize=20)
    
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.set_xlabel('\n\nX(m)')
    ax.set_ylabel('\n\nY(m)')
    ax.set_zlabel('     Z(m)     ')
    ax.xaxis.set_rotate_label(False) 
    ax.yaxis.set_rotate_label(False) 
    ax.zaxis.set_rotate_label(False) 
    ax.set_xlim3d(0, 0.6)
    ax.set_ylim3d(-0.8, -0.2)
    ax.set_zlim3d(0, 1)    
    colors = ["#000000", "#00ff00", "#88ff00", "#eeee00", "#ffaa00", "#ff5500", "#ff0000"]
    labels = ["Original", "10%", "30%", "50%", "60%", "70%", "80%"]

    # Plot Trajectories
    for traj in trajectories:
        idx = trajectories.index(traj)
        traj_len = len(traj)
        x = range(traj_len)

        traj_xyz = np.empty((traj_len, 3))
        for i in x:
            traj_xyz[i][0] = traj[i].translation.x
            traj_xyz[i][1] = traj[i].translation.y
            traj_xyz[i][2] = traj[i].translation.z

        ax.plot(traj_xyz[:,0], traj_xyz[:,1], traj_xyz[:,2], color=colors[idx], label=labels[idx])
        ax.legend()

    # Get camera TF
    camera_tf = None
    with open(BASE_DIR + '/curves/camera_tf.pickle', 'r') as f:
        camera_tf = pickle.load(f)
    
    # Plot Obstacles
    for bag_file in sorted(os.listdir(BASE_DIR + FOLDER)):
        if all(x in bag_file for x in["traj_real", ".bag"]):
            bag = rosbag.Bag(BASE_DIR + FOLDER + bag_file)

            obstacles = []
            for topic, msg, t in bag.read_messages():
                
                if topic == "/obstacles":
                    if msg.size > 0:
                        for i in range(msg.size):
                            obs_point = PointStamped(point=msg.centers[i])
                            obs_center = do_transform_point(obs_point, camera_tf).point
                            obstacles.append(obs_center)

            print(len(obstacles))
            # Constrain Obstacles
            for obstacle in list(obstacles):
                if 0.3 < obstacle.x < 0.6 and -0.7 < obstacle.y < 0.4:
                    continue
                else:
                    obstacles.remove(obstacle)

            print(len(obstacles))

            ax.scatter([o.x for o in obstacles], [o.y for o in obstacles], [o.z + 0.05 for o in obstacles])

    plt.show()


def trajSpeed():
    # Trajecotries lists
    traj_tests = [
        'traj_real_10',
        'traj_real_30',
        'traj_real_50',
        'traj_real_60',
        'traj_real_70',
        'traj_real_80',
    ]

    for test in traj_tests:
        bag = rosbag.Bag(BASE_DIR + FOLDER + test + '.bag')
        trajectory = openList(FOLDER + test + '.list')

        times = []
        for topic, msg, t in bag.read_messages():
            if topic == "/joint_states":
                if len(msg.name) == 6:
                    joints = []
                    for j_name in JOINT_NAMES:
                        joints.append(msg.position[msg.name.index(j_name)])

                    times.append(t)

        step = 50
        speeds = []
        for i in range(0, len(times)-step, step):
            time_diff = (times[i+step] - times[i]).to_sec()

            next_p = trajectory[i+step].translation
            next_point = np.array([next_p.x, next_p.y, next_p.z])

            cur_p = trajectory[i].translation
            cur_point = np.array([cur_p.x, cur_p.y, cur_p.z])

            speeds.append(np.linalg.norm(next_point - cur_point) / time_diff)

        print("%s Speed = %f" % (test, max(speeds)))
        

def trajMinDist():
    # Trajecotries lists
    traj_tests = [
        'traj_real_10',
        'traj_real_30',
        'traj_real_50',
        'traj_real_60',
        'traj_real_70',
        'traj_real_80',
    ]

    # Get camera TF
    camera_tf = None
    with open(BASE_DIR + '/curves/camera_tf.pickle', 'r') as f:
        camera_tf = pickle.load(f)

    # Get Obstacles
    obstacles = []
    for bag_file in sorted(os.listdir(BASE_DIR + FOLDER)):
        if all(x in bag_file for x in["traj_real", ".bag"]):
            bag = rosbag.Bag(BASE_DIR + FOLDER + bag_file)

            for topic, msg, t in bag.read_messages():
                
                if topic == "/obstacles":
                    if msg.size > 0:
                        for i in range(msg.size):
                            obs_point = PointStamped(point=msg.centers[i])
                            obs_center = do_transform_point(obs_point, camera_tf).point
                            obstacles.append(obs_center)
    # Constrain Obstacles
        for obstacle in list(obstacles):
            if 0.3 < obstacle.x < 0.6 and -0.7 < obstacle.y < 0.4:
                continue
            else:
                obstacles.remove(obstacle)

    # Get average obstacle position
    obstacle_mean = np.array([np.mean([o.x for o in obstacles]),
                              np.mean([o.y for o in obstacles]),
                              np.mean([o.z for o in obstacles])])

    # Get Minimum Distance btwn each Trajectory and Obstacles
    for test in traj_tests:
        trajectory = openList(FOLDER + test + '.list')
        
        distances = []
        for p in [x.translation for x in trajectory]:
            point = np.array([p.x, p.y, p.z])
            distances.append(np.linalg.norm(point - obstacle_mean))

        print("%s Min Dist = %f" % (test, min(distances)))


if __name__ == '__main__':
    # rospy.init_node('repulsion', anonymous=False)

    # trajGaz2List()
    # plotGazTraj()

    # trajReal2List()
    # plotRealTraj()

    trajSpeed()
    # trajMinDist()
    








        