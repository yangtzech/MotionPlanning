from dataclasses import dataclass
from typing import List, Optional

import numpy as np


@dataclass
class State:
    x: float
    y: float
    yaw: float
    v: float
    steer: Optional[float] = None


@dataclass
class Control:
    a: float  # acceleration
    delta: float  # steering angle (rad)


@dataclass
class Trajectory:
    states: List[State]
    ts: Optional[List[float]] = None

    def to_array(self) -> np.ndarray:
        """Convert trajectory to Nx5 numpy array: x,y,yaw,v,steer"""
        arr = []
        for s in self.states:
            steer = s.steer if s.steer is not None else 0.0
            arr.append([s.x, s.y, s.yaw, s.v, steer])
        return np.array(arr)
