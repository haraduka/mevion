import uuid
from pathlib import Path
import cv2
import numpy as np
import torch
from dataclasses import dataclass
from typing import List
from PIL import Image as PILImage
from datasets import Dataset, Features, Image, Sequence, Value
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
from lerobot.common.datasets.push_dataset_to_hub.utils import (
    concatenate_episodes,
    save_images_concurrently,
)
from lerobot.common.datasets.utils import (
    calculate_episode_data_index,
    hf_transform_to_torch,
)
import torch
# import torchvision

from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
from lerobot.common.policies.act.configuration_act import ACTConfig
from lerobot.common.policies.act.modeling_act import ACTPolicy
from lerobot.common.datasets.compute_stats import compute_stats

from pathlib import Path
import pickle
from mevion.lerobot.lerobot_episode import LeRobotEpisode

@dataclass
class DummyEpisode:
    images: np.ndarray
    states: np.ndarray
    actions: np.ndarray

    @classmethod
    def create(cls, T: int):
        image_list = []
        state_list = []
        action_list = []
        for _ in range(T):
            rand_image = np.random.randint(0, 256, size=(150, 85, 3), dtype=np.uint8)
            rand_state = np.random.rand(14)
            rand_action = np.random.rand(14)
            image_list.append(rand_image)
            state_list.append(rand_state)
            action_list.append(rand_action)
        images = np.array(image_list)
        states = np.array(state_list)
        actions = np.array(action_list)
        return cls(images, states, actions)

def ensure_bchw(imgs: torch.Tensor) -> torch.Tensor:
    # もし (B,1,C,H,W) の場合 → squeeze で (B,C,H,W)
    if imgs.ndim == 5 and imgs.size(1) == 1:
        imgs = imgs.squeeze(1)
    # もし (B,T,N_cam,C,H,W) の場合 → 先頭フレーム・先頭カメラを採用
    if imgs.ndim == 6:
        imgs = imgs[:, 0, 0]  # (B,C,H,W)
    if imgs.ndim != 4:
        raise RuntimeError(f"Expected (B,C,H,W), got {imgs.shape}")
    return imgs.contiguous()

def prepare_batch_for_act(batch: dict, device: torch.device) -> dict:
    assert isinstance(batch, dict)
    new_batch = {}
    for k, v in batch.items():
        if torch.is_tensor(v):
            v = v.to(device, non_blocking=True)
            # もし [B,1,D] みたいに余計な次元があれば潰す
            if v.ndim == 3 and v.size(1) == 1:
                v = v.squeeze(1)
            new_batch[k] = v
        else:
            new_batch[k] = v

    imgs = None
    if "observation.images" in new_batch and torch.is_tensor(new_batch["observation.images"]):
        imgs = new_batch["observation.images"]
    elif "observation.images.top" in new_batch and torch.is_tensor(new_batch["observation.images.top"]):
        imgs = new_batch["observation.images.top"]

    if imgs is not None:
        imgs = ensure_bchw(imgs)
        new_batch["observation.images"] = imgs
        new_batch["observation.images.top"] = imgs

    return new_batch

def convert_to_lerobot_dataset(
    episodes: List[LeRobotEpisode],
    fps: int,
    batch_size: int = 32,
    num_workers: int = 8) -> LeRobotDataset:
    # copied and tweaked from
    # https://github.com/ojh6404/imitator/blob/lerobot/imitator/scripts/lerobot_dataset_builder.py

    # create data dict
    ep_dicts = []
    for idx, ep in enumerate(episodes):
        T = len(ep.images)
        dones = torch.zeros(T, dtype=torch.bool)
        dones[-1] = True

        ep_dict = {}
        ep_dict["observation.images.top"] = [PILImage.fromarray(im) for im in ep.images]
        ep_dict["observation.state"] = torch.from_numpy(ep.states).float()
        ep_dict["action"] = torch.from_numpy(ep.actions).float()
        ep_dict["episode_index"] = torch.ones(T, dtype=torch.int64) * idx
        ep_dict["frame_index"] = torch.arange(0, T, 1)
        ep_dict["timestamp"] = torch.arange(0, T, 1) / fps
        ep_dict["next.done"] = dones
        ep_dicts.append(ep_dict)
    data_dict = concatenate_episodes(ep_dicts)
    total_frames = data_dict["frame_index"].shape[0]
    data_dict["index"] = torch.arange(0, total_frames, 1)

    # create haggingface dataset
    dim_state = len(episodes[0].states[0])
    dim_action = len(episodes[0].actions[0])
    features = Features(
        {
            "observation.images.top": Image(),
            "observation.state": Sequence(Value("float32"), length=dim_state),
            "action": Sequence(Value("float32"), length=dim_action),
            "episode_index": Value("int64"),
            "frame_index": Value("int64"),
            "timestamp": Value("float32"),
            "next.done": Value("bool"),
            "index": Value("int64"),
        }
    )
    hf_dataset = Dataset.from_dict(data_dict, features=features)
    hf_dataset.set_transform(hf_transform_to_torch)

    episode_data_index = calculate_episode_data_index(hf_dataset)
    info = {
        "fps": fps,
        "video": False,
    }
    lerobot_dataset = LeRobotDataset.from_preloaded(
        hf_dataset=hf_dataset,
        episode_data_index=episode_data_index,
        info=info,
    )
    print("compute stats")
    stats = compute_stats(lerobot_dataset, batch_size, num_workers)
    with open('data/stats.pkl', 'wb') as file:
        pickle.dump(stats, file)
    lerobot_dataset.stats = stats
    return lerobot_dataset


if __name__ == "__main__":
    output_directory = Path("outputs/train")
    output_directory.mkdir(parents=True, exist_ok=True)

    episode_list = []
    # for _ in range(30):
    #     episode_list.append(DummyEpisode.create(80))
    with open('/tmp/lerobot_episodes.pkl', 'rb') as file:
        episode_list = pickle.load(file)
    dataset = convert_to_lerobot_dataset(episode_list, 10)
    print(dataset)

    training_steps = 50000
    device = torch.device("cuda")
    log_freq = 500

    cfg = ACTConfig()
    cfg.input_shapes['observation.state'] = [dataset.features['observation.state'].length]
    cfg.output_shapes['action'] = [dataset.features['action'].length]
    cfg.chunk_size = 16
    cfg.n_action_steps = 16
    cfg.use_vae = False
    cfg.image_obs_keys = ["observation.images.top"]
    cfg.state_obs_keys = ["observation.state"]

    if hasattr(cfg, "n_obs_steps"):
        cfg.n_obs_steps = 1

    delta_timestamps = {
        "observation.images.top": [0.0],
        "observation.state": [0.0],
        "action": [i / dataset.fps for i in range(cfg.chunk_size)],
    }
    dataset.delta_timestamps = delta_timestamps

    policy = ACTPolicy(cfg, dataset_stats=dataset.stats)
    policy.train()
    policy.to(device)

    optimizer = torch.optim.AdamW(policy.parameters(), lr=1e-4)

    dataloader = torch.utils.data.DataLoader(
        dataset,
        num_workers=4,
        batch_size=64,
        shuffle=True,
        pin_memory=device != torch.device("cpu"),
        drop_last=True,
    )

    step = 0
    done = False
    while not done:
        for batch in dataloader:
            batch = prepare_batch_for_act(batch, device)
            output_dict = policy.forward(batch)
            loss = output_dict["loss"]
            loss.backward()
            optimizer.step()
            optimizer.zero_grad()

            if step % log_freq == 0:
                print(f"step: {step} loss: {loss.item():.3f}")
                policy.save_pretrained(output_directory)
            step += 1
            if step >= training_steps:
                done = True
                break
