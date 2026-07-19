import os
import skrobot

robot_model = skrobot.model.RobotModel()
robot_model.load_urdf_file(os.path.dirname(__file__) + '/../models/mevion_dae.urdf')
viewer = skrobot.viewers.PyrenderViewer(resolution=(320, 320))
viewer.add(robot_model)
viewer.show()
viewer.redraw()
print(robot_model.angle_vector())

import ipdb; ipdb.set_trace()
