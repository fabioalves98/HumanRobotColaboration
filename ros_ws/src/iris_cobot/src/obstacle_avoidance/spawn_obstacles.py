#!/usr/bin/env python
import rospkg, rospy
from math import sin, cos, radians
from urdf_parser_py.urdf import Sphere, URDF, Pose as URDFPose, Sphere
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SpawnModel, SetModelState

from iris_cobot.msg import Obstacles

BASE_DIR = rospkg.RosPack().get_path('iris_cobot')


def main():
    rospy.init_node('spawn_obstacles', anonymous=False)

    # Obstacles description
    model = URDF.from_xml_file(BASE_DIR + '/urdf/sphere.urdf')
    init_spawn = 10
    spheres = [
        # ('sphere_1', 0.05 , (0, 0, init_spawn)),
        # ('sphere_2', 0.05,  (0, 0, init_spawn)),
        # ('sphere_3', 0.1,  (1, 1, 0.5))
        ('sphere_right', 0.08, (0.9, 0.6, 0.55)),
        ('sphere_left', 0.08, (0.6, 0.9, 0.35)),
        ('sphere_center', 0.1, (0.8, 0.8, 0.2))

    ]

    # Spawn Models
    for sphere in spheres:
        print('Spawning - %s' % str(sphere))
        sphere_link = model.links[0]
        # Visuals
        sphere_link.visuals[0].geometry = Sphere(sphere[1])
        sphere_link.visuals[0].origin = URDFPose(sphere[2], (0, 0, 0))
        # Collisions
        sphere_link.collisions[0].origin = URDFPose(sphere[2], (0, 0, 0))
        sphere_link.collisions[0].geometry = Sphere(sphere[1])
        # Inertial
        sphere_link.inertial.origin = URDFPose((0, 0, 0), (0, 0, 0))

        # Spawm model
        spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        spawn_model_client(
            model_name = sphere[0],
            model_xml = model.to_xml_string(),
            robot_namespace = '/foo',
            reference_frame = 'world'
        )

    # Spawner service
    set_model_state_client = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    radius = 0.9

    # Obstacles publisher for accurate repulsion calculation
    obstacles_pub = rospy.Publisher('obstacles_fake', Obstacles, queue_size=1)
    
    # Switch models movement direction
    switch = True
    angles = []

    rate = rospy.Rate(50)
    # while not rospy.is_shutdown():

    #     if switch:
    #         angles = range(0, 900, 2)
    #     else:
    #         angles = range(900, 0, -2)

    #     for angle in angles:
    #         obstacles_centers = []
    #         obstacles_radiuses = []

    #         # Sphere 1
    #         x = radius * cos(radians(angle/10.0))
    #         y = radius * sin(radians(angle/10.0))
    #         z = 0.4

    #         model_state = ModelState()
    #         model_state.model_name = spheres[0][0]
    #         model_state.reference_frame = 'world'
    #         model_state.pose = Pose(Point(*[x, y, z - init_spawn]), Quaternion(*[0, 0, 0, 1]))
    #         set_model_state_client(model_state)

    #         obstacles_centers.append(Point(*[x, y, z]))
    #         obstacles_radiuses.append(spheres[0][1])
            
    #         # Sphere 2
    #         x = radius * cos(radians((900 - angle)/10.0))
    #         y = radius * sin(radians((900 - angle)/10.0))
    #         z =  0.8

    #         model_state = ModelState()
    #         model_state.model_name = spheres[1][0]
    #         model_state.reference_frame = 'world'
    #         model_state.pose = Pose(Point(*[x, y, z - init_spawn]), Quaternion(*[0, 0, 0, 1]))
    #         set_model_state_client(model_state)

    #         obstacles_centers.append(Point(*[x, y, z]))
    #         obstacles_radiuses.append(spheres[1][1])
    #         obstacles_pub.publish(Obstacles(size=2, centers=obstacles_centers, radiuses=obstacles_radiuses))
            
    #         rate.sleep()

    #     switch = not switch

    # rospy.spin()

if __name__ == "__main__":
    main()