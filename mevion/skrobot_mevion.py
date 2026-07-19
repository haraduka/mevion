from cached_property import cached_property
import os
import subprocess
import numpy as np

from skrobot.coordinates import CascadedCoords
from skrobot.model import RobotModel
from skrobot.models.urdf import RobotModelFromURDF

from ament_index_python.packages import get_package_share_directory

from mevion import parameters as P


class SkRobotSingleMevion(RobotModelFromURDF):
    def __init__(self, *args, **kwargs):
        super(SkRobotSingleMevion, self).__init__(*args, **kwargs)
        self.RL_end_coords = CascadedCoords(parent=self.RL_hand_link,
                                            name='RL_end_coords')
        self.RL_end_coords.translate([0, 0, 0.1])
        self.RL_end_coords.rotate(np.deg2rad(-90), axis='y')

        self.end_coords = [self.RL_end_coords]
        self.joint_names = [joint.name for joint in self.joint_list]
        self.params = P.SINGLE_PARAMS

    @cached_property
    def default_urdf_path(self):
        pkg_fullpath = get_package_share_directory("mevion")
        xacro_fullpath = os.path.join(pkg_fullpath, "models", "single_mevion.urdf.xacro")
        urdf_fullpath = xacro_fullpath.replace(".xacro", "")
        subprocess.call(["xacro", xacro_fullpath, "-o", urdf_fullpath])
        return urdf_fullpath

    def reset_pose(self):
        return self.arm.angle_vector(self.params["RESET_ANGLE"])

    def reset_manip_pose(self):
        return self.arm.angle_vector(self.params["RESET_MANIP_ANGLE"])

    @cached_property
    def arm(self):
        arm_links = [
                self.RL_shoulder_link,
                self.RL_upper_link,
                self.RL_lower1_link,
                self.RL_lower2_link,
                self.RL_wrist_link,
                self.RL_hand_link,
                self.RL_gripper_link_left
                ]
        arm_joints = []
        for link in arm_links:
            arm_joints.append(link.joint)
        robot = RobotModel(link_list=arm_links,
                       joint_list=arm_joints)
        robot.end_coords = self.RL_end_coords
        return robot


class SkRobotDualMevion(RobotModelFromURDF):
    def __init__(self, *args, **kwargs):
        super(SkRobotDualMevion, self).__init__(*args, **kwargs)
        self.RL_end_coords = CascadedCoords(parent=self.RL_hand_link,
                                              name='RL_end_coords')
        self.RL_end_coords.translate([0, 0, 0.1])
        self.RL_end_coords.rotate(np.deg2rad(-90), axis='z')
        self.RL_end_coords.rotate(np.deg2rad(90), axis='x')

        self.RF_end_coords = CascadedCoords(parent=self.RF_hand_link,
                                              name='RF_end_coords')
        self.RF_end_coords.translate([0, 0, 0.1])
        self.RF_end_coords.rotate(np.deg2rad(-90), axis='z')
        self.RF_end_coords.rotate(np.deg2rad(90), axis='x')

        self.end_coords = [self.RL_end_coords, self.RF_end_coords]
        self.joint_names = [joint.name for joint in self.joint_list]
        self.params = P.DUAL_PARAMS

    @cached_property
    def default_urdf_path(self):
        pkg_fullpath = get_package_share_directory("mevion")
        xacro_fullpath = os.path.join(pkg_fullpath, "models", "dual_mevion.urdf.xacro")
        urdf_fullpath = xacro_fullpath.replace(".xacro", "")
        subprocess.call(["xacro", xacro_fullpath, "-o", urdf_fullpath])
        return urdf_fullpath

    def reset_pose(self):
        return self.arms.angle_vector(self.params["RESET_ANGLE"])

    def reset_manip_pose(self):
        return self.arms.angle_vector(self.params["RESET_MANIP_ANGLE"])

    @cached_property
    def leader(self):
        leader_links = [self.RL_shoulder_link,
                        self.RL_upper_link,
                        self.RL_lower1_link,
                        self.RL_lower2_link,
                        self.RL_wrist_link,
                        self.RL_hand_link,
                        self.RL_gripper_link_left]
        leader_joints = []
        for link in leader_links:
            leader_joints.append(link.joint)
        robot = RobotModel(link_list=leader_links,
                       joint_list=leader_joints)
        robot.end_coords = self.RL_end_coords
        return robot

    @cached_property
    def follower(self):
        follower_links = [self.RF_shoulder_link,
                          self.RF_upper_link,
                          self.RF_lower1_link,
                          self.RF_lower2_link,
                          self.RF_wrist_link,
                          self.RF_hand_link,
                          self.RF_gripper_link_left]
        follower_joints = []
        for link in follower_links:
            follower_joints.append(link.joint)
        robot = RobotModel(link_list=follower_links,
                       joint_list=follower_joints)
        robot.end_coords = self.RF_end_coords
        return robot

    @cached_property
    def arms(self):
        arms_links = [
                      self.RL_shoulder_link,
                      self.RL_upper_link,
                      self.RL_lower1_link,
                      self.RL_lower2_link,
                      self.RL_wrist_link,
                      self.RL_hand_link,
                      self.RL_gripper_link_left,

                      self.RF_shoulder_link,
                      self.RF_upper_link,
                      self.RF_lower1_link,
                      self.RF_lower2_link,
                      self.RF_wrist_link,
                      self.RF_hand_link,
                      self.RF_gripper_link_left]
        arms_joints = []
        for link in arms_links:
            arms_joints.append(link.joint)
        robot = RobotModel(link_list=arms_links,
                       joint_list=arms_joints)
        robot.end_coords = self.end_coords
        return robot


class SkRobotQuadMevion(RobotModelFromURDF):
    def __init__(self, *args, **kwargs):
        super(SkRobotQuadMevion, self).__init__(*args, **kwargs)
        self.RL_end_coords = CascadedCoords(parent=self.RL_hand_link,
                                              name='RL_end_coords')
        self.RL_end_coords.translate([0, 0, 0.1])
        self.RL_end_coords.rotate(np.deg2rad(-90), axis='z')
        self.RL_end_coords.rotate(np.deg2rad(90), axis='x')

        self.RF_end_coords = CascadedCoords(parent=self.RF_hand_link,
                                              name='RF_end_coords')
        self.RF_end_coords.translate([0, 0, 0.1])
        self.RF_end_coords.rotate(np.deg2rad(-90), axis='z')
        self.RF_end_coords.rotate(np.deg2rad(90), axis='x')

        self.LL_end_coords = CascadedCoords(parent=self.LL_hand_link,
                                              name='LL_end_coords')
        self.LL_end_coords.translate([0, 0, 0.1])
        self.LL_end_coords.rotate(np.deg2rad(90), axis='z')
        self.LL_end_coords.rotate(np.deg2rad(90), axis='x')

        self.LF_end_coords = CascadedCoords(parent=self.LF_hand_link,
                                              name='LF_end_coords')
        self.LF_end_coords.translate([0, 0, 0.1])
        self.LF_end_coords.rotate(np.deg2rad(90), axis='z')
        self.LF_end_coords.rotate(np.deg2rad(90), axis='x')

        self.end_coords = [self.RL_end_coords, self.RF_end_coords,
                           self.LL_end_coords, self.LF_end_coords]
        self.joint_names = [joint.name for joint in self.joint_list]
        self.params = P.QUAD_PARAMS

    @cached_property
    def default_urdf_path(self):
        pkg_fullpath = get_package_share_directory("mevion")
        xacro_fullpath = os.path.join(pkg_fullpath, "models", "quad_mevion.urdf.xacro")
        urdf_fullpath = xacro_fullpath.replace(".xacro", "")
        subprocess.call(["xacro", xacro_fullpath, "-o", urdf_fullpath])
        return urdf_fullpath

    def reset_pose(self):
        return self.arms.angle_vector(self.params["RESET_ANGLE"])

    def reset_manip_pose(self):
        return self.arms.angle_vector(self.params["RESET_MANIP_ANGLE"])

    @cached_property
    def leader(self):
        leader_links = [
                self.RL_shoulder_link,
                self.RL_upper_link,
                self.RL_lower1_link,
                self.RL_lower2_link,
                self.RL_wrist_link,
                self.RL_hand_link,
                self.RL_gripper_link_left,

                self.LL_shoulder_link,
                self.LL_upper_link,
                self.LL_lower1_link,
                self.LL_lower2_link,
                self.LL_wrist_link,
                self.LL_hand_link,
                self.LL_gripper_link_left,
                ]
        leader_joints = []
        for link in leader_links:
            leader_joints.append(link.joint)
        robot = RobotModel(link_list=leader_links,
                       joint_list=leader_joints)
        robot.end_coords = [self.RL_end_coords, self.LL_end_coords]
        return robot

    @cached_property
    def follower(self):
        follower_links = [
                self.RF_shoulder_link,
                self.RF_upper_link,
                self.RF_lower1_link,
                self.RF_lower2_link,
                self.RF_wrist_link,
                self.RF_hand_link,
                self.RF_gripper_link_left,

                self.LF_shoulder_link,
                self.LF_upper_link,
                self.LF_lower1_link,
                self.LF_lower2_link,
                self.LF_wrist_link,
                self.LF_hand_link,
                self.LF_gripper_link_left,
                ]
        follower_joints = []
        for link in follower_links:
            follower_joints.append(link.joint)
        robot = RobotModel(link_list=follower_links,
                       joint_list=follower_joints)
        robot.end_coords = [self.RF_end_coords, self.LF_end_coords]
        return robot

    @cached_property
    def rarm(self):
        leader_links = [
                self.RL_shoulder_link,
                self.RL_upper_link,
                self.RL_lower1_link,
                self.RL_lower2_link,
                self.RL_wrist_link,
                self.RL_hand_link,
                self.RL_gripper_link_left,

                self.RF_shoulder_link,
                self.RF_upper_link,
                self.RF_lower1_link,
                self.RF_lower2_link,
                self.RF_wrist_link,
                self.RF_hand_link,
                self.RF_gripper_link_left,
                ]
        leader_joints = []
        for link in leader_links:
            leader_joints.append(link.joint)
        robot = RobotModel(link_list=leader_links,
                       joint_list=leader_joints)
        robot.end_coords = [self.RL_end_coords, self.RF_end_coords]
        return robot

    @cached_property
    def larm(self):
        leader_links = [
                self.LL_shoulder_link,
                self.LL_upper_link,
                self.LL_lower1_link,
                self.LL_lower2_link,
                self.LL_wrist_link,
                self.LL_hand_link,
                self.LL_gripper_link_left,

                self.LF_shoulder_link,
                self.LF_upper_link,
                self.LF_lower1_link,
                self.LF_lower2_link,
                self.LF_wrist_link,
                self.LF_hand_link,
                self.LF_gripper_link_left,
                ]
        leader_joints = []
        for link in leader_links:
            leader_joints.append(link.joint)
        robot = RobotModel(link_list=leader_links,
                       joint_list=leader_joints)
        robot.end_coords = [self.LL_end_coords, self.LF_end_coords]
        return robot

    @cached_property
    def arms(self):
        arms_links = [
                self.RL_shoulder_link,
                self.RL_upper_link,
                self.RL_lower1_link,
                self.RL_lower2_link,
                self.RL_wrist_link,
                self.RL_hand_link,
                self.RL_gripper_link_left,

                self.RF_shoulder_link,
                self.RF_upper_link,
                self.RF_lower1_link,
                self.RF_lower2_link,
                self.RF_wrist_link,
                self.RF_hand_link,
                self.RF_gripper_link_left,

                self.LL_shoulder_link,
                self.LL_upper_link,
                self.LL_lower1_link,
                self.LL_lower2_link,
                self.LL_wrist_link,
                self.LL_hand_link,
                self.LL_gripper_link_left,

                self.LF_shoulder_link,
                self.LF_upper_link,
                self.LF_lower1_link,
                self.LF_lower2_link,
                self.LF_wrist_link,
                self.LF_hand_link,
                self.LF_gripper_link_left,
                ]
        arms_joints = []
        for link in arms_links:
            arms_joints.append(link.joint)
        robot = RobotModel(link_list=arms_links,
                       joint_list=arms_joints)
        robot.end_coords = [self.RL_end_coords, self.RF_end_coords,
                           self.LL_end_coords, self.LF_end_coords]
        return robot
