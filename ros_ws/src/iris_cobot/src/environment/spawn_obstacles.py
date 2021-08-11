#!/usr/bin/env python
import rospkg, rospy
from urdf_parser_py.urdf import Sphere, URDF, Pose, Sphere
# from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel


BASE_DIR = rospkg.RosPack().get_path('iris_cobot')


def main():
    rospy.init_node('spawn_obstacles', anonymous=False)

    model = URDF.from_xml_file(BASE_DIR + '/urdf/sphere.urdf')

    spheres = [
        ('sphere_1', 0.3 , (1, 0, 0.3)),
        ('sphere_2', 0.25, (1, 1, 0.25)),
        ('sphere_3', 0.2 , (0, 1, 0.2))
    ]
    # Spawn 3 Models
    for sphere in spheres:
        print('Spawning - %s' % str(sphere))
        sphere_link = model.links[0]
        # Visuals
        sphere_link.visuals[0].geometry = Sphere(sphere[1])
        sphere_link.visuals[0].origin = Pose(sphere[2], (0, 0, 0))
        # Collisions
        sphere_link.collisions[0].origin = Pose(sphere[2], (0, 0, 0))
        sphere_link.collisions[0].geometry = Sphere(sphere[1])
        # Inertial
        sphere_link.inertial.origin = Pose(sphere[2], (0, 0, 0))

        # Spawm model
        spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        spawn_model_client(
            model_name = sphere[0],
            model_xml = model.to_xml_string(),
            robot_namespace = '/foo',
            reference_frame = 'world'
        )

    rospy.spin()


if __name__ == "__main__":
    main()