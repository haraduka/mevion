import argparse
import cv2
import sys
import glob
import numpy as np
import select
import os
from pathlib import Path
import threading
import pickle
import torch
from safetensors.torch import load_file
from typing import Any, Dict, List, Optional, Type

import rospy
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

from lerobot.common.policies.diffusion.modeling_diffusion import DiffusionPolicy
from lerobot.common.policies.diffusion.configuration_diffusion import DiffusionConfig


class CameraState:
    def __init__(self):
        self.rgb_data = None
        self.depth_data = None
        self.lock = threading.Lock()

class LeRobotFeedback(object):
    def __init__(self):
        rospy.init_node('lerobot_feedback', anonymous=True)

        self.camera_state = CameraState()

        self.bridge = CvBridge()

        rgb_sub   = message_filters.Subscriber('/camera/color/image_rect_color', Image, buff_size=2**24)
        depth_sub = message_filters.Subscriber('/camera/depth/image_rect_raw', Image, buff_size=2**24)
        ts = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub],
            queue_size=10,
            slop=0.1
        )
        ts.registerCallback(self.camera_callback)
        rospy.loginfo("lerobot feedback node setup done")

    def camera_callback(self, rgb_msg, depth_msg):
        try:
            color_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge error: {e}")
            return

        with self.camera_state.lock:
            self.camera_state.rgb_data = color_image
            self.camera_state.depth_data = depth_image

    def feedback(self, policy: DiffusionPolicy):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if select.select([sys.stdin,],[],[],0.0)[0]:
                input_char = sys.stdin.read(1)
                if input_char == '\n':
                    break

            with self.camera_state.lock:
                if self.camera_state.rgb_data is not None:
                    rgb = self.camera_state.rgb_data[:]
                    depth = self.camera_state.depth_data[:]
                else:
                    print("No camera data")
                    rate.sleep()
                    continue

            image = cv2.resize(rgb, (150, 85))
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            image = torch.from_numpy(image).permute(2, 0, 1).float()/255.
            image = image.unsqueeze(0)
            state = torch.from_numpy(np.array([0.3, 0.0, 0.1, 0])).float().unsqueeze(0)

            image = image.to("cuda")
            state = state.to("cuda")

            observation = {
                "observation.image": image,
                "observation.state": state
                }
            with torch.inference_mode():
                action = policy.select_action(observation)
            action_np = action.detach().cpu().numpy().flatten()
            print(action_np)
            rate.sleep()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--model_dir", type=str, default="outputs/train")
    args = parser.parse_args()
    model_dir: str = args.model_dir

    state_dict = load_file(os.path.join(model_dir, "model.safetensors"))
    torch.save(state_dict, os.path.join(model_dir, "model.pth"))

    lrf = LeRobotFeedback()
    with Path("./data/stats.pkl").open("rb") as f:
        stats = pickle.load(f)
    for key, value in stats.items():
        for key_sub, value_sub in value.items():
            stats[key][key_sub] = value_sub.to("cuda")
    cfg = DiffusionConfig()
    cfg.input_shapes['observation.state'] = [4]
    cfg.output_shapes['action'] = [4]
    # cfg.horizon = 1
    # cfg.n_action_steps = 1
    policy = DiffusionPolicy(cfg, dataset_stats=stats)
    policy = policy.to("cuda")
    policy.load_state_dict(torch.load(os.path.join(model_dir, "model.pth")))

    lrf.feedback(policy)
