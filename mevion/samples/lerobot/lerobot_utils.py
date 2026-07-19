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

import rospy
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

import cv2

from skrobot.coordinates import CascadedCoords
from mevion.samples.lerobot.lerobot_episode import LeRobotEpisode
import mevion.parameters as P

class CameraState:
    def __init__(self):
        self.rgb_data = None
        self.depth_data = None
        self.info_data = None
        self.lock = threading.Lock()

class LeRobotUtils:
    def __init__(self):
        if not rospy.core.is_initialized():
            rospy.init_node('lerobot_utils', anonymous=True)

        self.camera_state = CameraState()
        self.episodes = []

        self.bridge = CvBridge()

        info_sub  = message_filters.Subscriber('/camera/color/camera_info', CameraInfo, buff_size=2**24)
        rgb_sub   = message_filters.Subscriber('/camera/color/image_rect_color', Image, buff_size=2**24)
        depth_sub = message_filters.Subscriber('/camera/depth/image_rect_raw', Image, buff_size=2**24)
        ts = message_filters.ApproximateTimeSynchronizer(
            [info_sub, rgb_sub, depth_sub],
            queue_size=10,
            slop=0.1
        )
        ts.registerCallback(self.camera_callback)
        rospy.loginfo("lerobot utils setup done")

    def camera_callback(self, cam_info, rgb_msg, depth_msg):
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


    def save_episodes(self, filename="/tmp/lerobot_data.pkl"):
        with open(filename, 'wb') as f:
            pickle.dump(self.episodes, f)
            print(f"[SAVE] {filename} done")

    def load_episodes(self, filename="/tmp/lerobot_data.pkl"):
        with open(filename, 'rb') as f:
            self.episodes = pickle.load(f)
            print(f"[LOAD] {filename} done")

    def data_collect(self, m, kp_scale=0.003, kd_scale=0.02, torque_scale=0.85, collect_hz=10, hz=100):
        if m.mode != "dual" and m.mode != "quad":
            print("[unilateral_control] this function is valid in only dual and quad mode.")
            return
        l_arm_index = m.LIMB_INDEX["leader"]
        l_hand_index = m.HAND_INDEX["leader"]
        f_arm_index = m.LIMB_INDEX["follower"]
        f_hand_index = m.HAND_INDEX["follower"]
        m.robot_model.reset_manip_pose()
        m.send_angle_vector(m.robot_model.arms.angle_vector(), 3.0)
        print("unilateral control start")
        print("Please press Enter to start new episode, or press 'e' to exit")

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
                        print("Please press Enter to start new episode, or press 'e' to exit")
                elif not collecting and input_char == 'e':
                    print("[EXIT] finishing data collection")
                    break
            with m.robot_state.lock and m.robot_command.lock:
                comp_torque = m.calc_joint_torque(m.robot_state.angle)
                m.robot_command.torque = [torque_scale*torque for torque in comp_torque]
                for idx in l_arm_index:
                    if idx in l_hand_index:
                        vel = m.robot_state.velocity[idx]
                        m.robot_command.kp[idx] = 0.0
                        m.robot_command.kd[idx] = 0.0
                        m.robot_command.torque[idx] = m.compute_smooth_hand_torque(vel)
                    else:
                        m.robot_command.angle[idx] = m.robot_model.reset_manip_pose()[idx]
                        m.robot_command.kp[idx] = kp_scale*m.params["KP_GAIN"][idx]
                        m.robot_command.kd[idx] = kd_scale*m.params["KD_GAIN"][idx]
                for l_index, f_index in zip(l_arm_index, f_arm_index):
                    m.robot_command.angle[f_index] = m.robot_state.angle[l_index]
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
                    state = np.array(m.robot_state.angle[:])[f_arm_index]
                    action = np.array(m.robot_command.angle[:])[f_arm_index]
                episode.append((resized_rgb, state, action))
            collecting_count += 1
            rate.sleep()

        print("unilateral control stop")
        with m.robot_command.lock and m.robot_state.lock:
            m.robot_command.angle = m.robot_state.angle[:]
            m.robot_command.torque = [0.0]*m.N_JOINTS
            m.robot_command.kp = m.params["KP_GAIN"][:]
            m.robot_command.kd = m.params["KD_GAIN"][:]
        m.robot_model.reset_pose()
        m.send_angle_vector(m.robot_model.arms.angle_vector(), 3.0)

    def save_lerobot_episodes(self, filename="/tmp/lerobot_episodes.pkl"):
        episode_list = []
        for i_episode, episode in enumerate(self.episodes):
            images = []
            states = []
            actions = []
            print("=== New Episode ===")
            for image, state, action in episode:
                images.append(image)
                states.append(state)
                actions.append(action)
            episode_list.append(LeRobotEpisode(images=np.array(images), states=np.array(states), actions=np.array(actions)))
        with open(filename, 'wb') as f:
            pickle.dump(episode_list, f)
            print(f"[SAVE] {filename} done")

    def lerobot_feedback(self, m, model_dir="./outputs/train"):
        import torch
        from safetensors.torch import load_file
        from lerobot.common.policies.act.modeling_act import ACTPolicy
        from lerobot.common.policies.act.configuration_act import ACTConfig

        state_dict = load_file(os.path.join(model_dir, "model.safetensors"))
        torch.save(state_dict, os.path.join(model_dir, "model.pth"))

        with Path("./data/stats.pkl").open("rb") as f:
            stats = pickle.load(f)
        for key, value in stats.items():
            for key_sub, value_sub in value.items():
                stats[key][key_sub] = value_sub.to("cuda")

        cfg = ACTConfig()
        cfg.input_shapes["observation.images.top"] = [3, 85, 150]
        cfg.input_shapes["observation.state"] = [14]
        cfg.output_shapes["action"] = [14]
        cfg.chunk_size = 16
        cfg.n_action_steps = 8
        cfg.use_vae = False
        cfg.image_obs_keys = ["observation.images.top"]
        cfg.state_obs_keys = ["observation.state"]

        policy = ACTPolicy(cfg, dataset_stats=stats)
        policy = policy.to("cuda")
        policy.load_state_dict(torch.load(os.path.join(model_dir, "model.pth")))

        m.robot_model.reset_manip_pose()
        m.send_angle_vector(m.robot_model.follower.angle_vector(), 3.0, limb="follower")
        m.stop_grasp_pos(limb="follower", force=0, max_force=50, interpolation_time=1.0, kp=50)

        f_arm_index = m.LIMB_INDEX["follower"]
        f_hand_index = m.HAND_INDEX["follower"]

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            start_time = time.time()
            if select.select([sys.stdin], [], [], 0.0)[0]:
                input_char = sys.stdin.read(1)
                if input_char == "\n":
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
            image = torch.from_numpy(image).permute(2, 0, 1).float() / 255.0
            image = image.unsqueeze(0)

            with m.robot_state.lock:
                state = torch.from_numpy(np.array(m.robot_state.angle[:])[f_arm_index]).float().unsqueeze(0)

            image = image.to("cuda")
            state = state.to("cuda")

            observation = {
                "observation.images.top": image,
                "observation.state": state,
            }

            with torch.inference_mode():
                action = policy.select_action(observation)
            action = action.detach().cpu().numpy().flatten().tolist()

            m.robot_model.follower.angle_vector(action)
            m.send_angle_vector(
                m.robot_model.follower.angle_vector(),
                0.1,
                limb="follower",
                interpolation="linear",
            )

            end_time = time.time()
            print("loop time:", end_time - start_time)
            rate.sleep()

        m.robot_model.reset_pose()
        m.send_angle_vector(m.robot_model.follower.angle_vector(), 3.0, limb="follower")
        m.stop_grasp_pos(limb="follower", force=0, max_force=50, interpolation_time=1.0, kp=50)
