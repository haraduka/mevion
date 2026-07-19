#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
if os.environ.get("PYOPENGL_PLATFORM") is None:
    os.environ["PYOPENGL_PLATFORM"] = "egl" # when GPU is available. otherwise, ["pyglet", "osmesa"]

import time
import cv2
import argparse
import numpy as np
import skrobot
from skrobot.coordinates import CascadedCoords

from collections import OrderedDict
import pyrender
import urdfpy
from mevion.skrobot_mevion import SkRobotMevion

def create_raymond_lights():
    """
    Return raymond light nodes for the scene.
    """
    thetas = np.pi * np.array([1.0 / 6.0, 1.0 / 6.0, 1.0 / 6.0])
    phis = np.pi * np.array([0.0, 2.0 / 3.0, 4.0 / 3.0])

    nodes = []

    for phi, theta in zip(phis, thetas):
        xp = np.sin(theta) * np.cos(phi)
        yp = np.sin(theta) * np.sin(phi)
        zp = np.cos(theta)

        z = np.array([xp, yp, zp])
        z = z / np.linalg.norm(z)
        x = np.array([-z[1], z[0], 0.0])
        if np.linalg.norm(x) == 0:
            x = np.array([1.0, 0.0, 0.0])
        x = x / np.linalg.norm(x)
        y = np.cross(z, x)

        matrix = np.eye(4)
        matrix[:3, :3] = np.c_[x, y, z]
        nodes.append(pyrender.Node(light=pyrender.DirectionalLight(color=np.ones(3), intensity=1.0), matrix=matrix))

    return nodes

def main(args):
    #######   Render Robot   ##########
    image = np.zeros((480, 848, 3), dtype=np.uint8)
    height, width = image.shape[:2]
    renderer = pyrender.OffscreenRenderer(
        viewport_width=width, viewport_height=height
    )

    # Create scene and camera
    scene = pyrender.Scene(bg_color=[0.0, 0.0, 0.0, 0.0], ambient_light=(0.1, 0.1, 0.1))
    for light_node in create_raymond_lights():
        scene.add_node(light_node)
    camera = pyrender.IntrinsicsCamera(
        fx=435.0, fy=423.0, cx=(width - 1) / 2, cy=(height - 1) / 2
    )
    camera_node = scene.add(camera, pose=np.eye(4))

    # Load the skrobot model for IK
    skrobot_model = SkRobotMevion()
    skrobot_model.reset_manip_pose()
    skrobot_model.rarm.inverse_kinematics(CascadedCoords(pos=[0.3, -0.225, 0.2], rot=[0.0, 0.0, 0.0]), rotation_axis=True)

    # Load the robot model for Pyrender
    robot = urdfpy.URDF.load(os.path.join(os.path.dirname(__file__), "../models/dual_mevion_nonros.urdf"))
    joint_names = robot.actuated_joint_names # only actuated joints
    optical_camera_frame = CascadedCoords(pos=[0.3, 0.0, 0.4], parent=skrobot_model.world).rotate(np.pi, "z").rotate(-np.pi*150/188, "x")
    fk = robot.visual_trimesh_fk()  # Forward kinematics of visual meshes
    node_map = OrderedDict()

    # Set camera pose for render
    coordinate_transform = np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    camera_frame = optical_camera_frame.copy_worldcoords().T()
    camera_frame = camera_frame @ coordinate_transform # for Pyrender Convection, you need to apply this transformation
    camera_node.matrix = camera_frame

    # add robot mesh to scene
    for tm in fk:
        color = tm.visual.material.main_color
        avg_color = np.mean(color[:3])

        if avg_color < 60 or abs(avg_color-126) < 3: # black and grey gripper
            material = pyrender.MetallicRoughnessMaterial(
                baseColorFactor=[0.0, 0.0, 0.0, 1.0],
                metallicFactor=0.0,
                roughnessFactor=0.5
            )
        else:
            material = None

        pose = fk[tm]
        mesh = pyrender.Mesh.from_trimesh(tm, smooth=False, material=material)
        node = scene.add(mesh, pose=pose)
        node_map[tm] = node

    # apply forward kinematics to mesh
    joint_cfg = {} # Joint configuration from skrobot model
    for joint_name in joint_names:
        joint = getattr(skrobot_model, joint_name)
        joint_cfg[joint_name] = getattr(skrobot_model, joint_name).joint_angle()
    fk = robot.visual_trimesh_fk(cfg=joint_cfg)  # Forward kinematics
    for mesh in fk:
        pose = fk[mesh]
        node_map[mesh].matrix = pose

    # render robot on image
    import time
    start_time = time.time()
    rgba, depth = renderer.render(scene, flags=pyrender.RenderFlags.RGBA)
    rgb = cv2.cvtColor(rgba[:, :, :3].copy(), cv2.COLOR_RGB2BGR)
    alpha = rgba[:, :, 3].copy().astype(np.float32) / 255.0
    render_image = rgb * alpha[..., None] + image * (1 - alpha[..., None])
    render_image = render_image.astype(np.uint8)
    print("Render time: ", time.time() - start_time)

    # Save the rendered image
    output_path = "output.png"
    cv2.imwrite(output_path, render_image)

    # T_world_cam = optical_camera_frame.copy_worldcoords().T()
    # p_cam = np.array([0.29, 0, 0.4])
    # p_cam_h = np.hstack([p_cam, 1.0])       # shape (4,)

    # p_world_h = T_world_cam @ p_cam_h      # shape (4,)
    # p_world   = p_world_h[:3]              # [x_r, y_r, z_r]
    # print(p_world)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        type=str,
        default="rendered",
        help='Output file path'
    )
    args = parser.parse_args()
    main(args)
