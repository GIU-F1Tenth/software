from abc import ABC, abstractmethod


class Filter(ABC):
    @abstractmethod
    def filter_scan(self, ranges, angles, pose_x, pose_y, pose_yaw):
        ...
