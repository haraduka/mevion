
def circle_ik_test(m):
    import time
    import numpy as np

    m.robot_model.reset_pose()
    m.send_angle_vector(m.robot_model.arm.angle_vector(), 1.0, limb='arm')

    m.robot_model.reset_manip_pose()
    m.send_angle_vector(m.robot_model.arm.angle_vector(), 3.0, limb='arm')

    time.sleep(1.0)

    m.robot_model.arm.inverse_kinematics(m.make_coords([0.35, 0.0, 0.3], [0, 0, 0]), rotation_axis=True, limb='arm')
    m.send_angle_vector(m.robot_model.arm.angle_vector(), 3.0, limb='arm')
    time.sleep(1.0)

    print("circle ik start")

    for i in range(100):
        x = 0.35
        y = 0.0 + 0.1*np.sin(2*np.pi*i/100)
        z = 0.2 + 0.1*np.cos(2*np.pi*i/100)
        m.robot_model.arm.inverse_kinematics(m.make_coords([x, y, z], [0, 0, 0]), rotation_axis=True, limb='arm')
        m.robot_viewer.redraw()
        m.send_angle_vector(m.robot_model.arm.angle_vector(), 0.1, limb='arm', interpolation='linear')

    print("circle ik finished")
