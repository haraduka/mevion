from mevion.plainmp import MevionDualarmRobotSpec
from mevion.skrobot_mevion import SkRobotDualMevion

if __name__ == "__main__":
    model = SkRobotDualMevion()
    model.reset_manip_pose()
    spec = MevionDualarmRobotSpec()
    spec.debug_visualize_collision_spheres(model)
