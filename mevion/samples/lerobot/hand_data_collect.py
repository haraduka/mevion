import os
import time
import pickle
import sys
import select
import termios
import tty
import threading
import numpy as np
from pathlib import Path
from collections import OrderedDict

import rospy
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from image_geometry import PinholeCameraModel

import cv2
import mediapipe as mp

import urdfpy
import pyrender
import skrobot
from skrobot.coordinates import CascadedCoords
from mevion.skrobot_mevion import SkRobotMevion
from mevion.lerobot.lerobot_episode import LeRobotEpisode
import mevion.parameters as P

def get_key(timeout=0.1):
    """
    get one character without blocking
    return None if no key is pressed
    """
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([fd], [], [], timeout)
        if rlist:
            c = sys.stdin.read(1)
            return c
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


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


class CameraState:
    def __init__(self):
        self.rgb_data = None
        self.depth_data = None
        self.info_data = None
        self.lock = threading.Lock()

class HandDataCollect:
    def __init__(self):
        if not rospy.core.is_initialized():
            rospy.init_node('hand_data_collect', anonymous=True)

        self.camera_state = CameraState()
        self.episodes = []

        self.bridge = CvBridge()
        self.camera_model = PinholeCameraModel()
        self.cam_info_received = False

        self.is_setup_hand_mocap = False
        self.is_setup_render_robot = False

        info_sub  = message_filters.Subscriber('/camera/color/camera_info', CameraInfo, buff_size=2**24)
        rgb_sub   = message_filters.Subscriber('/camera/color/image_rect_color', Image, buff_size=2**24)
        depth_sub = message_filters.Subscriber('/camera/depth/image_rect_raw', Image, buff_size=2**24)
        ts = message_filters.ApproximateTimeSynchronizer(
            [info_sub, rgb_sub, depth_sub],
            queue_size=10,
            slop=0.1
        )
        ts.registerCallback(self.camera_callback)
        rospy.loginfo("hand data collection node setup done")

    def setup_all(self): # should be called before mevion.setup_sim/real()
        self.setup_hand_mocap()
        self.setup_render_robot()

    def setup_hand_mocap(self):
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        print("[INFO] setup hand mocap done")
        self.is_setup_hand_mocap = True

    def camera_callback(self, cam_info, rgb_msg, depth_msg):
        if not self.cam_info_received:
            self.camera_model.fromCameraInfo(cam_info)
            self.cam_info_received = True

        try:
            color_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge error: {e}")
            return

        with self.camera_state.lock:
            self.camera_state.rgb_data = color_image
            self.camera_state.depth_data = depth_image
            self.camera_state.info_data = cam_info

    def hand_data_collect(self):
        episode = []
        collecting = False
        rate = rospy.Rate(10)

        print("=== Controls ===")
        print(" Enter : start/stop collecting")
        print(" + : save episode")
        print(" - : discard episode")
        print(" e : exit")
        print("================")

        while not rospy.is_shutdown():
            key = get_key(0.1)

            if key in ('\n', '\r'):
                if collecting:
                    collecting = False
                    print("[STOP] collection")
                else:
                    episode = []
                    collecting = True
                    print("[START] new episode")
            elif key == '+':
                if not collecting and episode:
                    self.episodes.append(episode)
                    print(f"[SAVE] {len(episode)} episodes collected")
                    print(f"[SAVE] episodes count = {len(self.episodes)}")
                else:
                    print("[WARN] no episode to save")
            elif key == '-':
                if not collecting:
                    episode = []
                    print("[DISCARD] current episode")
            elif key == 'e':
                print("[EXIT] finishing data collection")
                break

            if collecting:
                with self.camera_state.lock:
                    if self.camera_state.rgb_data is not None:
                        rgb = self.camera_state.rgb_data.copy()
                        depth = self.camera_state.depth_data.copy()
                        info = self.camera_state.info_data
                        episode.append((rgb, depth, info))

            rate.sleep()

        rospy.loginfo("Data collection node shutting down")

    def process_hand_mocap(self, color_image, depth_image, cam_info, debug=False):
        if not self.is_setup_hand_mocap:
            self.setup_hand_mocap()
        if not self.cam_info_received:
            self.camera_model.fromCameraInfo(cam_info)
            self.cam_info_received = True
        h, w = cam_info.height, cam_info.width

        rgb_for_mp = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb_for_mp)
        if not results.multi_hand_landmarks:
            return None, None

        hand_points = []  # List[List[(x,y,z)]]

        for hand_id, hand_lms in enumerate(results.multi_hand_landmarks):
            points3d = []
            for idx, lm in enumerate(hand_lms.landmark):
                u = int(lm.x * w); v = int(lm.y * h)
                u, v = np.clip(u, 0, w-1), np.clip(v, 0, h-1)
                depth = float(depth_image[v, u])
                if np.isnan(depth) or depth <= 0:
                    points3d.append(None)
                    continue
                x = (u - self.camera_model.cx()) * depth / self.camera_model.fx()
                y = (v - self.camera_model.cy()) * depth / self.camera_model.fy()
                z = depth
                points3d.append((x/1000.0, y/1000., z/1000.))
            hand_points.append(points3d)

        palm_centers = [[] for _ in range(len(hand_points))]
        open_close = [[] for _ in range(len(hand_points))]

        for hand_id, pts in enumerate(hand_points):
            palm_idxs = [0, 5, 9, 13, 17]  # Wrist, each MCP
            valid = [pts[i] for i in palm_idxs if pts[i] is not None]
            if not valid:
                rospy.logwarn(f"Hand{hand_id}: cannot calculate palm center")
                paldm_centers[hand_id] = None
            else:
                pc = np.mean(valid, axis=0)  # average of (x,y,z)
                palm_centers[hand_id] = pc
            if debug:
                print(f"=== Hand {hand_id} ===")
                print(f"Palm center: x={pc[0]:.3f}, y={pc[1]:.3f}, z={pc[2]:.3f}")

            tip_idxs = [4, 8, 12, 16, 20]
            dists = []
            for ti in tip_idxs:
                if pts[ti] is None:
                    continue
                d = np.linalg.norm(np.array(pts[ti]) - pc)
                dists.append(d)
            if dists:
                avg_dist = float(np.mean(dists))
                state = "OPEN" if avg_dist > 0.06 else "CLOSE"
                if debug:
                    print(f"Finger-tip distance={avg_dist:.3f} m --> {state}")
            else:
                if debug:
                    print(f"Finger-tip distance cannot calculate")
                state = None
            open_close[hand_id] = state
        return palm_centers, open_close

    def save_episodes(self, filename="/tmp/hand_data.pkl"):
        with open(filename, 'wb') as f:
            pickle.dump(self.episodes, f)
            print(f"[SAVE] {filename} done")

    def load_episodes(self, filename="/tmp/hand_data.pkl"):
        with open(filename, 'rb') as f:
            self.episodes = pickle.load(f)
            print(f"[LOAD] {filename} done")

    def process_all_episodes(self): # test function
        for episode in self.episodes:
            print("=== New Episode ===")
            for rgb, depth, cam_info in episode:
                palm_centers, open_close = self.process_hand_mocap(rgb, depth, cam_info)
                if palm_centers is None:
                    print("No hand detected")
                    continue
                self.render_robot(rgb, depth, cam_info, palm_centers[0], open_close[0], debug=False)
                print(f"Palm centers: {palm_centers}")
                print(f"Open/Close states: {open_close}")

    def setup_render_robot(self, width=848, height=480, skrobot_model=None):
        self.renderer = pyrender.OffscreenRenderer(viewport_width=width, viewport_height=height)

        self.scene = pyrender.Scene(bg_color=[0.0, 0.0, 0.0, 0.0], ambient_light=(0.1, 0.1, 0.1))
        for light_node in create_raymond_lights():
            self.scene.add_node(light_node)
        camera = pyrender.IntrinsicsCamera(
            fx=435.0, fy=423.0, cx=(width - 1) / 2, cy=(height - 1) / 2
        )
        camera_node = self.scene.add(camera, pose=np.eye(4))

        # Load the skrobot model for IK
        if skrobot_model is not None:
            self.skrobot_model = skrobot_model
        else:
            self.skrobot_model = SkRobotMevion()
        self.skrobot_model.reset_manip_pose()

        # Load the robot model for Pyrender
        self.robot = urdfpy.URDF.load(os.path.join(os.path.dirname(__file__), "../../models/dual_mevion_nonros.urdf"))
        self.optical_camera_frame = CascadedCoords(pos=[0.29, -0.03, 0.34], parent=self.skrobot_model.world).rotate(np.pi, "z").rotate(-np.pi*150/180, "x")
        fk = self.robot.visual_trimesh_fk()  # Forward kinematics of visual meshes
        self.node_map = OrderedDict()

        # Set camera pose for render
        coordinate_transform = np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        camera_frame = self.optical_camera_frame.copy_worldcoords().T()
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
            node = self.scene.add(mesh, pose=pose)
            self.node_map[tm] = node

        print("[INFO] setup render robot done")
        self.is_setup_render_robot = True

    def calc_hand_worldpos(self, palm_center):
        if not self.is_setup_render_robot:
            self.setup_render_robot()

        T_world_cam = self.optical_camera_frame.copy_worldcoords().T()
        p_cam = np.array(palm_center)
        p_cam_h = np.hstack([p_cam, 1.0]) # shape (4,)

        p_world_h = T_world_cam @ p_cam_h # shape (4,)
        p_world = p_world_h[:3] # [x_r, y_r, z_r]

        return p_world

    def render_robot(self, color_image, depth_image, cam_info, palm_center, open_close, debug=False):
        if not self.is_setup_render_robot:
            self.setup_render_robot()

        p_world = self.calc_hand_worldpos(palm_center)

        self.skrobot_model.rarm.inverse_kinematics(CascadedCoords(pos=p_world, rot=[0.0, 1.3, 0.0]), rotation_axis=True)
        if open_close == "OPEN":
            self.skrobot_model.R_gripper_prismatic_left.joint_angle(0.0415)
        else:
            self.skrobot_model.R_gripper_prismatic_left.joint_angle(0.0)

        # apply forward kinematics to mesh
        joint_cfg = {} # Joint configuration from skrobot model
        for joint_name in self.robot.actuated_joint_names: # only actuated joints
            joint = getattr(self.skrobot_model, joint_name)
            joint_cfg[joint_name] = getattr(self.skrobot_model, joint_name).joint_angle()
        fk = self.robot.visual_trimesh_fk(cfg=joint_cfg)  # Forward kinematics
        for mesh in fk:
            pose = fk[mesh]
            self.node_map[mesh].matrix = pose

        # render robot on image
        rgba, depth = self.renderer.render(self.scene, flags=pyrender.RenderFlags.RGBA)
        rgb = cv2.cvtColor(rgba[:, :, :3].copy(), cv2.COLOR_RGB2BGR)
        alpha = rgba[:, :, 3].copy().astype(np.float32) / 255.0
        render_image = rgb * alpha[..., None] + color_image * (1 - alpha[..., None])
        render_image = render_image.astype(np.uint8)
        cv2.imwrite("output_robot.png", render_image)

        # segmentation image
        mask = alpha > 0.5  # shape: (H, W)
        seg = np.zeros_like(color_image)       # black image
        seg[mask] = [255, 255, 255]            # make mask white
        cv2.imwrite("output_segmentation.png", seg)

        return render_image, depth

    def save_hand_lerobot_episodes(self, filename="/tmp/hand_lerobot_episodes.pkl", render=True, save_video=False):
        episode_list = []
        for i_episode, episode in enumerate(self.episodes):
            render_images = []
            images = []
            states = []
            actions = []
            prev_action = None
            print("=== New Episode ===")
            for rgb, depth, cam_info in episode:
                palm_centers, open_close = self.process_hand_mocap(rgb, depth, cam_info, debug=False)
                if palm_centers is None:
                    print("No hand detected")
                    continue
                if not (palm_centers[0] is not None):
                    print("Hand palm pos cannot be calculated")
                    continue
                p_world = self.calc_hand_worldpos(palm_centers[0]).tolist()
                p_world[0] = min(max(0.15, p_world[0]), 0.4)
                p_world[1] = min(max(-0.1, p_world[1] + 0.225), 0.1)
                p_world[2] = min(max(0.1, p_world[2]-0.05), 0.3)
                action = p_world + [1] if open_close[0] == "CLOSE" else p_world + [0] # close means 1
                if prev_action is None:
                    prev_action = action
                    continue
                if render:
                    render_image, depth = self.render_robot(rgb, depth, cam_info, palm_centers[0], open_close[0], debug=False)
                else:
                    render_image = rgb
                resized_rgb = cv2.resize(render_image, (150, 85))
                if save_video:
                    render_images.append(render_image)
                images.append(resized_rgb)
                states.append(prev_action)
                print(action)
                actions.append(action)
                prev_action = action[:]
            episode_list.append(LeRobotEpisode(images=np.array(images), states=np.array(states), actions=np.array(actions)))

            if save_video:
                height, width, channels = render_images[0].shape
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # You can also use 'XVID' or 'avc1'
                output_path = "output-" + str(i_episode) + ".mp4"
                writer = cv2.VideoWriter(output_path, fourcc, 10, (width, height)) # fps=10
                for img in render_images:
                    # Ensure the image is in uint8 format
                    if img.dtype != np.uint8:
                        img = np.clip(img, 0, 255).astype(np.uint8)
                    writer.write(img)
                writer.release()
                print(f"Video saved to {output_path}")
        with open(filename, 'wb') as f:
            pickle.dump(episode_list, f)
            print(f"[SAVE] {filename} done")

    def hand_control(self, m):
        if not self.is_setup_hand_mocap:
            self.setup_hand_mocap()
        if not self.is_setup_render_robot:
            self.setup_render_robot(skrobot_model=m.robot_model)

        m.robot_model.larm.inverse_kinematics(CascadedCoords(pos=[0.3, 0.225, 0.1], rot=[0.0, 1.3, 0.0]), rotation_axis=True)
        m.send_angle_vector(m.robot_model.larm.angle_vector(), 3.0, limb="larm")
        m.stop_grasp_pos(limb="larm", force=0, max_force=50, interpolation_time=1.0, kp=50)

        smoothed_p_world = [0.3, 0.225, 0.1]
        smoothed_open_close = 0.0415

        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            # if enter is pressed, break with non-blocking
            if select.select([sys.stdin,],[],[],0.0)[0]:
                input_char = sys.stdin.read(1)
                if input_char == '\n':
                    break

            with self.camera_state.lock:
                if self.camera_state.rgb_data is not None:
                    rgb = self.camera_state.rgb_data[:]
                    depth = self.camera_state.depth_data[:]
                    info = self.camera_state.info_data
                else:
                    print("No camera data")
                    continue
            if not (rgb is not None):
                print("No camera data")
                continue
            palm_centers, open_close = self.process_hand_mocap(rgb, depth, info)
            if not (palm_centers is not None):
                print("No hand detected")
                continue
            if not (palm_centers[0] is not None):
                print("Hand palm pos cannot be calculated")
                continue
            p_world = self.calc_hand_worldpos(palm_centers[0])
            p_world[0] = min(max(0.15, p_world[0]), 0.4)
            p_world[1] = min(max(-0.1+0.225, p_world[1] + 0.45), 0.1+0.225)
            p_world[2] = min(max(0.1, p_world[2]-0.05), 0.3)
            smoothed_p_world = (np.array(smoothed_p_world) * 0.7 + np.array(p_world) * 0.3).tolist()
            print(palm_centers[0], open_close[0], p_world, smoothed_p_world)

            m.robot_model.larm.inverse_kinematics(CascadedCoords(pos=smoothed_p_world, rot=[0.0, 1.3, 0.0]), rotation_axis=True)
            if open_close[0] is not None:
                if open_close[0] == "OPEN":
                    smoothed_open_close = smoothed_open_close * 0.7 + 0.0415 * 0.3
                    m.robot_model.L_gripper_prismatic_left.joint_angle(smoothed_open_close)
                else:
                    smoothed_open_close = smoothed_open_close * 0.7 + 0.0 * 0.3
                    m.robot_model.L_gripper_prismatic_left.joint_angle(smoothed_open_close)
            m.send_angle_vector(m.robot_model.larm.angle_vector(), 0.1, limb="larm", interpolation="linear")

            rate.sleep()

        m.robot_model.reset_pose()
        m.send_angle_vector(m.robot_model.larm.angle_vector(), 3.0, limb="larm")

    def hand_lerobot_feedback(self, m, model_dir="./outputs/train"):
        import torch
        from safetensors.torch import load_file
        from lerobot.common.policies.diffusion.modeling_diffusion import DiffusionPolicy
        from lerobot.common.policies.diffusion.configuration_diffusion import DiffusionConfig

        state_dict = load_file(os.path.join(model_dir, "model.safetensors"))
        torch.save(state_dict, os.path.join(model_dir, "model.pth"))

        with Path("./data/stats.pkl").open("rb") as f:
            stats = pickle.load(f)
        for key, value in stats.items():
            for key_sub, value_sub in value.items():
                stats[key][key_sub] = value_sub.to("cuda")
        cfg = DiffusionConfig()
        cfg.input_shapes['observation.state'] = [4]
        cfg.output_shapes['action'] = [4]
        cfg.horizon = 16
        cfg.n_action_steps = 8
        policy = DiffusionPolicy(cfg, dataset_stats=stats)
        policy = policy.to("cuda")
        policy.load_state_dict(torch.load(os.path.join(model_dir, "model.pth")))

        m.robot_model.rarm.inverse_kinematics(CascadedCoords(pos=[0.3, -0.225, 0.2], rot=[0.0, 1.3, 0.0]), rotation_axis=True)
        m.send_angle_vector(m.robot_model.rarm.angle_vector(), 3.0, limb="rarm")
        m.stop_grasp_pos(limb="rarm", force=0, max_force=50, interpolation_time=1.0, kp=50)

        prev_action = [0.3, 0.0, 0.2, 0]

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            start_time = time.time()
            if select.select([sys.stdin,],[],[],0.0)[0]:
                input_char = sys.stdin.read(1)
                if input_char == '\n':
                    break

            with self.camera_state.lock:
                if self.camera_state.rgb_data is not None:
                    rgb = self.camera_state.rgb_data[:]
                else:
                    print("No camera data")
                    rate.sleep()
                    continue

            image = cv2.resize(rgb, (150, 85))
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            image = torch.from_numpy(image).permute(2, 0, 1).float()/255.
            image = image.unsqueeze(0)
            # with m.robot_state.lock:
            #     m.robot_model.arms.angle_vector(m.robot_state.angle)
            state = torch.from_numpy(np.array(prev_action)).float().unsqueeze(0)

            image = image.to("cuda")
            state = state.to("cuda")

            observation = {
                "observation.image": image,
                "observation.state": state
                }
            with torch.inference_mode():
                action = policy.select_action(observation)
            action = action.detach().cpu().numpy().flatten().tolist()

            action[0] = min(max(0.15, action[0]), 0.4)
            action[1] = min(max(-0.1, action[1]), 0.1)
            action[2] = min(max(0.1, action[2]), 0.3)
            action[3] = min(max(0.0, action[3]), 1.0)
            action_smoothed = (np.array(prev_action) * 0.0 + np.array(action) * 1.0).tolist()
            hand_pos = (1.0-action[3]) * 0.0415
            target_pos = action[:3]
            target_pos[1] = target_pos[1] - 0.225
            m.robot_model.rarm.inverse_kinematics(CascadedCoords(pos=target_pos, rot=[0.0, 1.3, 0.0]), rotation_axis=True)
            m.robot_model.R_gripper_prismatic_left.joint_angle(hand_pos)
            m.send_angle_vector(m.robot_model.rarm.angle_vector(), 0.1, limb="rarm", interpolation="linear")

            prev_action = action[:]
            end_time = time.time()
            print(end_time - start_time)
            rate.sleep()

        m.robot_model.reset_pose()
        m.send_angle_vector(m.robot_model.rarm.angle_vector(), 3.0, limb="rarm")

    def unilateral_data_collect(self, m, kp_scale=0.003, kd_scale=0.02, torque_scale=0.8, collect_hz=10, hz=100):
        larm_index = P.LIMB_INDEX["larm"]
        lhand_index = P.HAND_INDEX["larm"]
        rarm_index = P.LIMB_INDEX["rarm"]
        rhand_index = P.HAND_INDEX["rarm"]
        m.robot_model.reset_manip_pose()
        m.send_angle_vector(m.robot_model.arms.angle_vector(), 3.0)
        print("unilateral control start")

        episode = []
        collecting = False
        collecting_count = 0
        rate = rospy.Rate(hz)
        while True:
            if select.select([sys.stdin,],[],[],0.0)[0]:
                input_char = sys.stdin.read(1)
                if input_char == '\n':
                    if not collecting:
                        collecting = True
                        episode = []
                        print("[START] new episode")
                        print("next epsiode is {}".format(len(self.episodes)+1))
                    else:
                        collecting = False
                        self.episodes.append(episode)
                        print("[STOP] collection")
                        print("[SAVE] episode count = {}".format(len(episode)))
                elif not collecting and input_char == 'e':
                    print("[EXIT] finishing data collection")
                    break
            with m.robot_state.lock and m.robot_command.lock:
                comp_torque = m.calc_joint_torque(m.robot_state.angle)
                m.robot_command.torque = [torque_scale*torque for torque in comp_torque]
                for idx in larm_index:
                    if idx in lhand_index:
                        vel = m.robot_state.velocity[idx]
                        m.robot_command.kp[idx] = 0.0
                        m.robot_command.kd[idx] = 0.0
                        m.robot_command.torque[idx] = m.compute_smooth_hand_torque(vel)
                    else:
                        m.robot_command.angle[idx] = m.robot_model.reset_manip_pose()[idx]
                        m.robot_command.kp[idx] = kp_scale*P.KP_GAIN[idx]
                        m.robot_command.kd[idx] = kd_scale*P.KD_GAIN[idx]
                for l_index, r_index in zip(larm_index, rarm_index):
                    m.robot_command.angle[r_index] = m.robot_state.angle[l_index]
            if collecting and collecting_count % (hz // collect_hz) == 0:
                with self.camera_state.lock:
                    if self.camera_state.rgb_data is not None:
                        rgb = self.camera_state.rgb_data[:]
                    else:
                        print("No camera data")
                        continue
                    # resized_rgb = np.zeros((85, 150, 3), dtype=np.uint8)
                    resized_rgb = cv2.resize(rgb, (150, 85))
                with m.robot_state.lock:
                    state = np.array(m.robot_state.angle[:])[rarm_index]
                    action = np.array(m.robot_command.angle[:])[rarm_index]
                episode.append((resized_rgb, state, action))
            collecting_count += 1
            rate.sleep()

        print("unilateral control stop")
        with m.robot_command.lock and m.robot_state.lock:
            m.robot_command.angle = m.robot_state.angle[:]
            m.robot_command.torque = [0.0]*P.N_JOINTS
            m.robot_command.kp = P.KP_GAIN[:]
            m.robot_command.kd = P.KD_GAIN[:]
        m.robot_model.reset_pose()
        m.send_angle_vector(m.robot_model.arms.angle_vector(), 3.0)

    def save_unilateral_lerobot_episodes(self, filename="/tmp/unilateral_lerobot_episodes.pkl"):
        episode_list = []
        for i_episode, episode in enumerate(self.episodes):
            images = []
            states = []
            actions = []
            print("=== New Episode ===")
            for image, state, action in episode:
                images.append(image)
                states.append(state)
                print(action)
                actions.append(action)
            episode_list.append(LeRobotEpisode(images=np.array(images), states=np.array(states), actions=np.array(actions)))
        with open(filename, 'wb') as f:
            pickle.dump(episode_list, f)
            print(f"[SAVE] {filename} done")

    def unilateral_lerobot_feedback(self, m, model_dir="./outputs/train"):
        import torch
        from safetensors.torch import load_file
        from lerobot.common.policies.diffusion.modeling_diffusion import DiffusionPolicy
        from lerobot.common.policies.diffusion.configuration_diffusion import DiffusionConfig

        state_dict = load_file(os.path.join(model_dir, "model.safetensors"))
        torch.save(state_dict, os.path.join(model_dir, "model.pth"))

        with Path("./data/stats.pkl").open("rb") as f:
            stats = pickle.load(f)
        for key, value in stats.items():
            for key_sub, value_sub in value.items():
                stats[key][key_sub] = value_sub.to("cuda")
        cfg = DiffusionConfig()
        cfg.input_shapes['observation.image'] = [3, 85, 150]
        cfg.input_shapes['observation.state'] = [7]
        cfg.output_shapes['action'] = [7]
        cfg.crop_shape = (75, 135)
        cfg.pretrained_backbone_weights = "ResNet18_Weights.IMAGENET1K_V1"
        cfg.use_group_norm = False
        cfg.horizon = 16
        cfg.n_action_steps = 8
        policy = DiffusionPolicy(cfg, dataset_stats=stats)
        policy = policy.to("cuda")
        policy.load_state_dict(torch.load(os.path.join(model_dir, "model.pth")))

        m.robot_model.rarm.inverse_kinematics(CascadedCoords(pos=[0.2, -0.225, 0.35], rot=[0.0, 1.57, 0.0]), rotation_axis=True)
        m.send_angle_vector(m.robot_model.rarm.angle_vector(), 3.0, limb="rarm")
        m.stop_grasp_pos(limb="rarm", force=0, max_force=50, interpolation_time=1.0, kp=50)

        rarm_index = P.LIMB_INDEX["rarm"]
        rhand_index = P.HAND_INDEX["rarm"]

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            start_time = time.time()
            if select.select([sys.stdin,],[],[],0.0)[0]:
                input_char = sys.stdin.read(1)
                if input_char == '\n':
                    break

            with self.camera_state.lock:
                if self.camera_state.rgb_data is not None:
                    rgb = self.camera_state.rgb_data[:]
                else:
                    print("No camera data")
                    rate.sleep()
                    continue

            image = cv2.resize(rgb, (150, 85))
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            image = torch.from_numpy(image).permute(2, 0, 1).float()/255.
            image = image.unsqueeze(0)
            with m.robot_state.lock:
                state = torch.from_numpy(np.array(m.robot_state.angle[:])[rarm_index]).float().unsqueeze(0)

            image = image.to("cuda")
            state = state.to("cuda")

            observation = {
                "observation.image": image,
                "observation.state": state
                }
            with torch.inference_mode():
                action = policy.select_action(observation)
            action = action.detach().cpu().numpy().flatten().tolist()

            m.robot_model.rarm.angle_vector(action)
            m.send_angle_vector(m.robot_model.rarm.angle_vector(), 0.1, limb="rarm", interpolation="linear")

            end_time = time.time()
            print(end_time - start_time)
            rate.sleep()

        m.robot_model.reset_pose()
        m.send_angle_vector(m.robot_model.rarm.angle_vector(), 3.0, limb="rarm")
        m.stop_grasp_pos(limb="rarm", force=0, max_force=50, interpolation_time=1.0, kp=50)
