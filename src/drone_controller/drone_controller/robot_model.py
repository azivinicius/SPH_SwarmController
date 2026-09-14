"""Plain state container for one drone -- no ROS, no ties to any single node."""

from dataclasses import dataclass, field
from typing import List

import numpy as np


@dataclass
class Robot:
    name: str = ""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0

    @property
    def position_3d(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z], dtype=float)

    @property
    def velocity_3d(self) -> np.ndarray:
        return np.array([self.vx, self.vy, self.vz], dtype=float)

    def update_state(self, x, y, z, vx, vy, vz) -> None:
        self.x, self.y, self.z = float(x), float(y), float(z)
        self.vx, self.vy, self.vz = float(vx), float(vy), float(vz)