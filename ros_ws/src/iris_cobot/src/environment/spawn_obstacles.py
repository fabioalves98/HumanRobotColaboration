#!/usr/bin/env python
import rospkg, rospy
from math import sin, cos, radians
from urdf_parser_py.urdf import Sphere, URDF, Pose as URDFPose, Sphere
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SpawnModel, SetModelState

BASE_DIR = rospkg.RosPack().get_path('iris_cobot')


def main():
    rospy.init_node('spawn_obstacles', anonymous=False)

    model = URDF.from_xml_file(BASE_DIR + '/urdf/sphere.urdf')
    init_spawn = 10
    spheres = [
        ('sphere_1', 0.1 , (0, 0, init_spawn)),
        ('sphere_2', 0.1,  (0, 0, init_spawn)),
        ('sphere_3', 0.1,  (1, 1, 0.1))
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

    set_model_state_client = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    radius = 1

    # Change models pose
    switch = True
    angles = []

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        if switch:
            angles = range(200, 700)
        else:
            angles = range(700, 200, -1)

        for angle in angles:
            # Sphere 1
            x = radius * cos(radians(angle/10.0))
            y = radius * sin(radians(angle/10.0))

            model_state = ModelState()
            model_state.model_name = 'sphere_1'
            model_state.reference_frame = 'world'
            model_state.pose = Pose(Point(*[x, y, 0.3 - init_spawn]), Quaternion(*[0, 0, 0, 1]))
            
            set_model_state_client(model_state)

            # Sphere 2
            x = radius * cos(radians((900 - angle)/10.0))
            y = radius * sin(radians((900 - angle)/10.0))

            model_state = ModelState()
            model_state.model_name = 'sphere_2'
            model_state.reference_frame = 'world'
            model_state.pose = Pose(Point(*[x, y, 0.6 - init_spawn]), Quaternion(*[0, 0, 0, 1]))
            
            set_model_state_client(model_state)

            rate.sleep()

        switch = not switch

    rospy.spin()

if __name__ == "__main__":
    main()