#!/usr/bin/env python
import rospy, rospkg
from math import radians
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker

from sami.arm import Arm

def main():
    rospy.init_node('globe', anonymous=False)

    arm = Arm('ur10e_moveit', group='manipulator', joint_positions_filename="positions.yaml")
    arm.velocity = 1

    poses_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
    markers = []

    # Possible combinations of angles
    angles = [-180, -135, -90, -45, 0, 45, 90, 135]
    forbidden = [(45, -180),  (45, -135),
                 (90, -180),  (90, -135), (90, 135),
                 (135, -180),  (135, 135)]

    # Color array for markers
    colors = [
        [0, 0, 1], # -180    - Blue     - Vertical Right 
        [0, 1, 1], # -135    - Cyan     - Diagonal Right Top
        [0, 1, 0], # -90     - Green    - Horizontal Top 
        [1, 1, 0], # -45     - Yellow   - Diagonal Left Top
        [1, 0, 0], # 0       - Red      - Vertical Left 
        [1, 0, 1], # 45      - Pink     - Diagonal Left Bottom
        [1, 1, 1], # 90      - White    - Horizontal Bottom 
        [0, 0, 0]  # 135     - Black    - Diagonal Right Bottom
    ]

    idx = 0
    for w_1 in angles:
        w_idx = angles.index(w_1)
        for w_2 in angles:
            if (w_1, w_2) not in forbidden:
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
                marker.color = ColorRGBA(*(colors[w_idx] + [1]))

                markers.append(marker)

                poses_pub.publish(markers)


if __name__ == "__main__":
    main()