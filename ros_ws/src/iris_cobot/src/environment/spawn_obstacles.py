#!/usr/bin/env python
import rospkg, rospy
from urdf_parser_py.urdf import Sphere, URDF, Pose, Sphere
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
from gazebo_msgs.msg import ODEPhysics
from gazebo_msgs.srv import SpawnModel, SetPhysicsProperties


BASE_DIR = rospkg.RosPack().get_path('iris_cobot')


def main():
    rospy.init_node('spawn_obstacles', anonymous=False)

    model = URDF.from_xml_file(BASE_DIR + '/urdf/sphere.urdf')

    spheres = [
        ('sphere_1', 0.3 , (1, 0, 0.3)),
        ('sphere_2', 0.1, (1, 1, 0.1)),
        ('sphere_3', 0.15, (1.3, 1.3, 0.15)),
        ('sphere_4', 0.05, (1.3, 1, 0.3)),
        ('sphere_5', 0.05, (1, 1.3, 0.3)),
        ('sphere_6', 0.2 , (0, 1, 0.2))
    ]
    # Change gravity
    # set_gravity = rospy.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)
    # time_step = Float64(0.001)
    # max_update_rate = Float64(1000.0)
    # gravity = Vector3()
    # gravity.x = 0.0
    # gravity.y = 0.0
    # gravity.z = 0.0
    # ode_config = ODEPhysics()
    # ode_config.auto_disable_bodies = False
    # ode_config.sor_pgs_precon_iters = 0
    # ode_config.sor_pgs_iters = 50
    # ode_config.sor_pgs_w = 1.3
    # ode_config.sor_pgs_rms_error_tol = 0.0
    # ode_config.contact_surface_layer = 0.001
    # ode_config.contact_max_correcting_vel = 0.0
    # ode_config.cfm = 0.0
    # ode_config.erp = 0.2
    # ode_config.max_contacts = 20
    # set_gravity(time_step.data, max_update_rate.data, gravity, ode_config)

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

    # rospy.spin()


if __name__ == "__main__":
    main()