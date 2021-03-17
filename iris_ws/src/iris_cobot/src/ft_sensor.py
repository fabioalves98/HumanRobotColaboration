

def main():
    rospy.init_node('test', anonymous=False)
    signal.signal(signal.SIGINT, signal_handler)

    global arm

    arm = Arm('ur10e_moveit', group='manipulator', joint_positions_filename="positions.yaml")
    arm.velocity = 0.2

    # Test
    arm.move_pose([0.5, 0.5, 0.8, 0, 0, 0])

    m_x_pub = rospy.Publisher('mx_marker', Marker, queue_size=10)
    m_p_x_pub = rospy.Publisher('mpx_marker', Marker, queue_size=10)
    m_y_pub = rospy.Publisher('my_marker', Marker, queue_size=10)
    m_p_y_pub = rospy.Publisher('mpy_marker', Marker, queue_size=10)
    m_z_pub = rospy.Publisher('mz_marker', Marker, queue_size=10)
    m_p_z_pub = rospy.Publisher('mpz_marker', Marker, queue_size=10)
    m_g_pub = rospy.Publisher('mg_marker', Marker, queue_size=10)
    m_p_g_pub = rospy.Publisher('mpg_marker', Marker, queue_size=10)

    wrench_pub = rospy.Publisher('wrench', WrenchStamped, queue_size=1)

    while not rospy.is_shutdown():
        origin = [0.5, 0.5, 0.5]

        # Arrow x
        m_x = Marker()
        m_x.header.frame_id = "base_link"
        m_x.type = m_x.ARROW
        m_x.action = m_x.ADD
        m_x.scale = Vector3(*[0.2, 0.02, 0.02])

        rot = quaternion_from_euler(0, 0, radians(-90))
        arm_ori = orientation_to_list(arm.get_pose().orientation)
        m_x_ori = quaternion_multiply(arm_ori, rot)
        m_x.pose.orientation = list_to_orientation(m_x_ori)

        m_x.pose.position  = Point(*origin)
        m_x.color = ColorRGBA(*[1, 0, 0, 1])

        # Arrow y
        m_y = Marker()
        m_y.header.frame_id = "base_link"
        m_y.type = m_y.ARROW
        m_y.action = m_y.ADD
        m_y.scale = Vector3(*[0.2, 0.02, 0.02])

        rot = quaternion_from_euler(0, radians(90), 0)
        arm_ori = orientation_to_list(arm.get_pose().orientation)
        m_y_ori = quaternion_multiply(arm_ori, rot)
        m_y.pose.orientation = list_to_orientation(m_y_ori)

        m_y.pose.position = Point(*origin)
        m_y.color = ColorRGBA(*[0, 1, 0, 1])

        # Arrow z
        m_z = Marker()
        m_z.header.frame_id = "base_link"
        m_z.type = m_z.ARROW
        m_z.action = m_z.ADD
        m_z.scale = Vector3(*[0.2, 0.02, 0.02])

        m_z.pose.orientation = arm.get_pose().orientation
        m_z.pose.position = Point(*origin)
        m_z.color = ColorRGBA(*[0, 0, 1, 1])

        # Gravity
        m_g = Marker()
        m_g.header.frame_id = "base_link"
        m_g.type = m_g.ARROW
        m_g.action = m_g.ADD
        m_g.scale = Vector3(*[0.2, 0.02, 0.02])
        m_g.pose.orientation = list_to_orientation(quaternion_from_euler(0, radians(90), 0))
        m_g.pose.position = Point(*origin)
        m_g.color = ColorRGBA(*[1, 0, 0.5, 1])

        # Auxiliary position for vector creation
        aux_p = PoseStamped()
        aux_p.header.frame_id = "base_link"
        aux_p.pose.position = Point(*[0.3, 0, 0])
        aux_p.pose.orientation = Quaternion(*[0, 0, 0, 1])

        # Point x
        t_x = TransformStamped()
        t_x.header.frame_id = "base_link"
        t_x.transform.translation = Vector3(*origin)
        t_x.transform.rotation = m_x.pose.orientation

        p_x_new = do_transform_pose(aux_p, t_x)

        m_p_x = Marker()
        m_p_x.header.frame_id = "base_link"
        m_p_x.type = m_p_x.SPHERE
        m_p_x.action = m_p_x.ADD
        m_p_x.scale = Vector3(*[0.03, 0.03, 0.03])
        m_p_x.pose = p_x_new.pose
        m_p_x.color = ColorRGBA(*[1, 0, 0, 1])

        # Point y 
        t_y = TransformStamped()
        t_y.header.frame_id = "base_link"
        t_y.transform.translation = Vector3(*origin)
        t_y.transform.rotation = m_y.pose.orientation

        p_y_new = do_transform_pose(aux_p, t_y)

        m_p_y = Marker()
        m_p_y.header.frame_id = "base_link"
        m_p_y.type = m_p_y.SPHERE
        m_p_y.action = m_p_y.ADD
        m_p_y.scale = Vector3(*[0.03, 0.03, 0.03])
        m_p_y.pose = p_y_new.pose
        m_p_y.color = ColorRGBA(*[0, 1, 0, 1])

        # Point z
        t_z = TransformStamped()
        t_z.header.frame_id = "base_link"
        t_z.transform.translation = Vector3(*origin)
        t_z.transform.rotation = m_z.pose.orientation

        p_z_new = do_transform_pose(aux_p, t_z)

        m_p_z = Marker()
        m_p_z.header.frame_id = "base_link"
        m_p_z.type = m_p_z.SPHERE
        m_p_z.action = m_p_z.ADD
        m_p_z.scale = Vector3(*[0.03, 0.03, 0.03])
        m_p_z.pose = p_z_new.pose
        m_p_z.color = ColorRGBA(*[0, 0, 1, 1])

        # Point g
        t_g = TransformStamped()
        t_g.header.frame_id = "base_link"
        t_g.transform.translation = Vector3(*origin)
        t_g.transform.rotation = m_g.pose.orientation

        p_g_new = do_transform_pose(aux_p, t_g)

        m_p_g = Marker()
        m_p_g.header.frame_id = "base_link"
        m_p_g.type = m_p_g.SPHERE
        m_p_g.action = m_p_g.ADD
        m_p_g.scale = Vector3(*[0.03, 0.03, 0.03])
        m_p_g.pose = p_g_new.pose
        m_p_g.color = ColorRGBA(*[1, 0, 0.5, 1])

        # Publishers
        m_x_pub.publish(m_x)
        m_p_x_pub.publish(m_p_x)
        m_y_pub.publish(m_y)
        m_p_y_pub.publish(m_p_y)
        m_z_pub.publish(m_z)
        m_p_z_pub.publish(m_p_z)
        m_g_pub.publish(m_g)
        m_p_g_pub.publish(m_p_g)

        # Vectors
        v_x = (np.array(point_to_list(p_x_new.pose.position)) - np.array(origin))
        v_y = (np.array(point_to_list(p_y_new.pose.position)) - np.array(origin))
        v_z = (np.array(point_to_list(p_z_new.pose.position)) - np.array(origin))
        v_g = (np.array(point_to_list(p_g_new.pose.position)) - np.array(origin))

        # Normalize
        v_x = v_x / np.linalg.norm(v_x)
        v_y = v_y / np.linalg.norm(v_y)
        v_z = v_z / np.linalg.norm(v_z)
        v_g = v_g / np.linalg.norm(v_g)
        
        # Obtain force
        f_x = np.inner(v_x, v_g) * 15
        f_y = np.inner(v_y, v_g) * 15
        f_z = np.inner(v_z, v_g) * 15

        # Wrench pub
        wrench = WrenchStamped()
        wrench.header.frame_id = "base_link"
        wrench.wrench.force = Vector3(*[f_x, f_y, f_z])

        wrench_pub.publish(wrench)

        rospy.sleep(0.002)

if __name__ == "__main__":
    main()