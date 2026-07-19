import os
import pprint
import subprocess
import urdf_parser_py.urdf as urdf
from ament_index_python.packages import get_package_share_directory

from mevion import parameters as P


def get_urdf_joint_params(urdf_path, joint_names):
    if isinstance(urdf_path, str) and urdf_path.endswith('.urdf'):
        robot_urdf = open(urdf_path).read()

    robot = urdf.Robot.from_xml_string(robot_urdf)
    # joint_params = {}
    joint_params = [None]*len(joint_names)

    for joint in robot.joints:
        if joint.name in joint_names and (joint.type == 'revolute' or joint.type == 'continuous' or joint.type == 'prismatic'):
            if joint.limit:
                index = joint_names.index(joint.name)
                joint_params[index] = (joint.limit.lower, joint.limit.upper, joint.limit.effort, joint.limit.velocity)

    return joint_params

def test_get_urdf_joint_params():
    pkg_fullpath = get_package_share_directory("mevion")
    xacro_fullpath = os.path.join(pkg_fullpath, "models", "quad_mevion.urdf.xacro")
    urdf_fullpath = xacro_fullpath.replace(".xacro", "")
    subprocess.call(["xacro", xacro_fullpath, "-o", urdf_fullpath])
    pprint.pprint(get_urdf_joint_params(urdf_fullpath, P.JOINT_NAME))

def interpolate_linear(start, end, t):
    return start + (end - start) * t

def interpolate_minjerk(start, end, t):
    t3 = t**3
    t4 = t**4
    t5 = t**5
    return (start * (-6*t5 + 15*t4 - 10*t3 + 1) +
            end * (6*t5 - 15*t4 + 10*t3))


if __name__ == '__main__':
    print("# test_get_urdf_joint_params")
    test_get_urdf_joint_params()

