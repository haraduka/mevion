import time

import numpy as np
from mevion.plainmp import MevionLarmRobotSpec
from mevion.skrobot_mevion import SkRobotDualMevion
from skrobot.coordinates import Coordinates
from skrobot.model.primitives import Axis, Box, Cylinder
from skrobot.viewers import PyrenderViewer

from plainmp.ik import solve_ik
from plainmp.ompl_solver import OMPLSolver, OMPLSolverConfig
from plainmp.problem import Problem
from plainmp.psdf import GroundSDF, UnionSDF
from plainmp.utils import primitive_to_plainmp_sdf

print("preparing model and specs")
model = SkRobotDualMevion()
model.reset_manip_pose()
ms = MevionLarmRobotSpec()
ms.reflect_skrobot_model_to_kin(model)

cylinder = Cylinder(radius=0.05, height=0.5, pos=[0.25, 0.15, 0.25], face_colors=[255, 0, 0, 200])
cylinder_sdf = primitive_to_plainmp_sdf(cylinder)
ceiling = Box([1.0, 1.0, 0.05], pos=[0.4, 0.0, 0.6], face_colors=[0, 255, 0, 150])  # for safety
ceiling_sdf = primitive_to_plainmp_sdf(ceiling)
ground_sdf = GroundSDF(0.0)
sdf = UnionSDF([ground_sdf, cylinder_sdf, ceiling_sdf])

collision_cst = ms.create_collision_const(self_collision=True)
collision_cst.set_sdf(sdf)
lb, ub = ms.angle_bounds()
start_pose = np.array([0.4, 0.0, 0.1, 0.0, 0.0, 0.0])
goal_pose = np.array([0.4, 0.3, 0.1, 0.0, 0.0, 0.0])

print("compute collision aware IK to determine the start configuration")
pose_const_start = ms.create_gripper_pose_const(start_pose)
q_now = ms.extract_skrobot_model_q(model)
ret = solve_ik(pose_const_start, collision_cst, lb, ub, q_seed=q_now)
assert ret.success
q_start = ret.q

print("compute collision aware IK to determine the goal configuration")
pose_const_goal = ms.create_gripper_pose_const(goal_pose)
ret = solve_ik(pose_const_goal, collision_cst, lb, ub, q_seed=q_now)
assert ret.success
q_goal = ret.q

print("solve path planning")
solver = OMPLSolver(OMPLSolverConfig(shortcut=True))
resolution = [0.05, 0.05, 0.05, 0.1, 0.1, 0.15]
problem = Problem(q_start, lb, ub, q_goal, collision_cst, None, resolution=resolution)
result = solver.solve(problem)
assert result.traj is not None

print("visualize result")
v = PyrenderViewer()
v.add(model)
ground_box = Box([1.0, 1.0, 0.05])  # ground
ground_box.translate([0.4, 0.0, -0.025])
v.add(ground_box)
v.add(cylinder)
v.add(ceiling)
v.add(Axis.from_coords(Coordinates(pos=start_pose[:3])))
v.add(Axis.from_coords(Coordinates(pos=goal_pose[:3])))

v.show()

for q in result.traj.resample(200):
    ms.set_skrobot_model_state(model, q)
    time.sleep(0.02)
    v.redraw()
time.sleep(100)
