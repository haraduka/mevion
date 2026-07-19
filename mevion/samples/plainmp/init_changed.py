import subprocess
import tempfile
import time
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Optional

import numpy as np
from skrobot.model.primitives import Sphere
from skrobot.viewers import PyrenderViewer

from plainmp.constraint import LinkPoseCst
from plainmp.robot_spec import RobotSpec


class MevionRobotSpecBase(RobotSpec):
    def urdf_path_override(self) -> Path:
        xacro_path = self.project_root / "models" / "dual_mevion.urdf.xacro"
        return xacro_path

    @property
    def project_root(self) -> Path:
        return Path(__file__).parent.parent.parent.parent


class MevionLarmRobotSpec(MevionRobotSpecBase):
    def __init__(self):
        conf_path = Path(__file__).parent / "mevion_larm.yaml"
        super().__init__(conf_path)

    def create_gripper_pose_const(self, link_pose: np.ndarray) -> LinkPoseCst:
        return self.create_pose_const(["larm_end_coords"], [link_pose])


class MevionRarmRobotSpec(MevionRobotSpecBase):
    def __init__(self):
        conf_path = Path(__file__).parent / "mevion_rarm.yaml"
        super().__init__(conf_path)

    def create_gripper_pose_const(self, link_pose: np.ndarray) -> LinkPoseCst:
        return self.create_pose_const(["rarm_end_coords"], [link_pose])


class MevionDualarmRobotSpec(MevionRobotSpecBase):
    def __init__(self):
        conf_path = Path(__file__).parent / "mevion_dualarm.yaml"
        super().__init__(conf_path)

    def create_gripper_pose_const(
        self, rarm_pose: np.ndarray, larm_pose: np.ndarray
    ) -> LinkPoseCst:
        return self.create_pose_const(
            ["rarm_end_coords", "larm_end_coords"], [rarm_pose, larm_pose]
        )
