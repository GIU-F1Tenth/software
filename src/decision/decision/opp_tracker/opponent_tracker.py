from collections import deque
from dataclasses import dataclass


@dataclass(frozen=True)
class OpponentObservation:
    x: float
    y: float
    z: float
    stamp: float
    frame_id: str


def centroid(points):
    count = len(points)
    if count == 0:
        raise ValueError("cannot compute the centroid of an empty point set")
    return tuple(sum(axis) / count for axis in zip(*points))


class OpponentCenterTracker:
    def __init__(self, max_history_size=500):
        self._history = deque(maxlen=max_history_size)

    def add_observation(self, observation):
        self._history.append(observation)
        return observation

    @property
    def latest(self):
        return self._history[-1] if self._history else None

    @property
    def history(self):
        return list(self._history)

    def clear(self):
        self._history.clear()

    def __len__(self):
        return len(self._history)
