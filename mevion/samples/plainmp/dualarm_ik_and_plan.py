import time

import numpy as np
from mevion.plainmp import MevionDualarmRobotSpec
from mevion.skrobot_mevion import SkRobotDualMevion
from skrobot.coordinates import Coordinates
from skrobot.model.primitives import Axis, Box
from skrobot.viewers import PyrenderViewer

from plainmp.ik import solve_ik
from plainmp.ompl_solver import OMPLSolver, OMPLSolverConfig
from plainmp.problem import Problem
from plainmp.psdf import GroundSDF

if __name__ == "__main__":
    model = SkRobotDualMevion()
    ms = MevionDualarmRobotSpec()
    lb, ub = ms.angle_bounds()
    collision_cst = ms.create_collision_const(self_collision=True)
    ground_sdf = GroundSDF(0.0)
    collision_cst.set_sdf(ground_sdf)

    print("start configuration is reset manip pose")
    model.reset_manip_pose()
    q_start = ms.extract_skrobot_model_q(model)

    print("solve IK to determine goal")
    follower_goal = np.array([0.3, 0.15, 0.1, 0.0, 0.0, 0.0])
    leader_goal = np.array([0.3, -0.15, 0.45, 0.0, 0.0, 0.0])
    pose_const = ms.create_gripper_pose_const(follower_goal, leader_goal)
    ret = solve_ik(pose_const, None, lb, ub, q_seed=q_start)
    assert ret.success
    q_goal = ret.q

    print("solve path planning")
    solver = OMPLSolver(OMPLSolverConfig(shortcut=True))
    resolution = [0.05, 0.05, 0.05, 0.1, 0.1, 0.15, 0.05, 0.05, 0.05, 0.1, 0.1, 0.15]
    problem = Problem(q_start, lb, ub, q_goal, collision_cst, None, resolution=resolution)
    result = solver.solve(problem)
    assert result.traj is not None

    v = PyrenderViewer()
    ms.set_skrobot_model_state(model, ret.q)
    v.add(model)
    v.add(Axis.from_coords(Coordinates(pos=follower_goal[:3])))
    v.add(Axis.from_coords(Coordinates(pos=leader_goal[:3])))
    ground_box = Box([1.0, 1.0, 0.05], pos=[0.4, 0.0, -0.025])
    v.add(ground_box)

    v.show()

    for q in result.traj.resample(300):
        ms.set_skrobot_model_state(model, q)
        time.sleep(0.02)
        v.redraw()
    time.sleep(100)
