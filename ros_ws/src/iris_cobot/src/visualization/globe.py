#!/usr/bin/env python
import rospy, rospkg, pickle
from math import radians
from geometry_msgs.msg import Vector3, Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker

from sami.arm import Arm

BASE_DIR = rospkg.RosPack().get_path('iris_cobot')

# Possible combinations of angles
ANGLES = [-180, -135, -90, -45, 0, 45, 90, 135]
FORBIDDEN = [(45, -180),  (45, -135),
             (90, -180),  (90, -135), (90, 135),
             (135, -180), (135, 135)]

# Color array for markers
COLORS = [
    [0, 0, 1], # -180    - Blue     - Vertical Right 
    [0, 1, 1], # -135    - Cyan     - Diagonal Right Top
    [0, 1, 0], # -90     - Green    - Horizontal Top 
    [1, 1, 0], # -45     - Yellow   - Diagonal Left Top
    [1, 0, 0], # 0       - Red      - Vertical Left 
    [1, 0, 1], # 45      - Pink     - Diagonal Left Bottom
    [1, 1, 1], # 90      - White    - Horizontal Bottom 
    [0, 0, 0]  # 135     - Black    - Diagonal Right Bottom
]

def vectorFromQuaternion(quaternion):
    pass


def createMarkers(arm):
    markers = []

    idx = 0
    for w_1 in ANGLES:
        w_idx = ANGLES.index(w_1)
        for w_2 in ANGLES:
            if (w_1, w_2) not in FORBIDDEN:
                print('Idx - %d - %d - %d' % (idx, w_1, w_2))
                
                arm.move_joints([0, radians(-90), 0, radians(w_1), radians(w_2), 0])

                marker = Marker()
                marker.header.frame_id = "base_link"
                marker.header.stamp = rospy.Time()
                marker.id = idx
                idx += 1
                marker.type = marker.ARROW
                marker.action = marker.ADD
                marker.scale = Vector3(*[0.1, 0.01, 0.01])
                ee_pose = arm.get_pose()
                marker.pose = ee_pose
                print('Orientation - %s' % str(ee_pose.orientation))
                marker.color = ColorRGBA(*(COLORS[w_idx] + [1]))

                markers.append(marker)

    # Save markers
    with open(BASE_DIR + '/curves/markers.pickle', 'w') as f:
        pickle.dump(markers, f)
    
    return markers

def loadMarkers():
    markers = []

    with open(BASE_DIR + '/curves/markers.pickle') as f:
        markers = pickle.load(f)

    return markers


def main():
    rospy.init_node('globe', anonymous=False)

    arm = Arm('ur10e_moveit', group='manipulator', joint_positions_filename="positions.yaml")
    arm.velocity = 1

    poses_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)

    # markers = createMarkers(arm)

    # Load markers and do stuff
    markers = loadMarkers()
    for marker in markers:
        marker.pose.position = Point(*[-1, 0, 1.5])
        marker.header.stamp = rospy.Time()

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        poses_pub.publish(markers)
        rate.sleep()

if __name__ == "__main__":
    main()