import numpy as np
from dataclasses import dataclass

@dataclass
class LeRobotEpisode:
    images: np.ndarray
    states: np.ndarray
    actions: np.ndarray
