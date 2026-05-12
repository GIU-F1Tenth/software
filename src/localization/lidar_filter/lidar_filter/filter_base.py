from abc import ABC, abstractmethod

import numpy as np


class Filter(ABC):
    @abstractmethod
    def filter_scan(self, ranges, angles, pose_x, pose_y, pose_yaw) -> np.ndarray:
        ...
