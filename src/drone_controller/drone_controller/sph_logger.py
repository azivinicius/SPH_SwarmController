"""Per-robot CSV telemetry logger, ported from the reference project.

log_dir defaults to a path inside ./src (already bind-mounted into the
container), so files land on your HOST at ~/SPH_Project/src/logs/ in
real time and survive container teardown -- same idea as the original.
"""

import csv
import os
import time
from typing import Optional, TextIO


class SphMetricsLogger:
    _FIELDNAMES = [
        "t", "robot_name",
        "rho", "pressure", "sound_speed",
        "x", "y", "z", "vx", "vy", "vz",
        "sph_ax", "sph_ay", "sph_az",
        "cmd_ax", "cmd_ay", "cmd_az",   # always 0 here -- this node never commands
        "n_known_neighbors", "n_total_neighbors",
        "fx", "fy", "fz",               # reserved, not computed yet (potential-field force)
        "real_x", "real_y", "real_z",
        "real_vx", "real_vy", "real_vz",
    ]

    def __init__(
        self,
        robot_name: str,
        log_dir: str = "/root/dev_ws/src/logs",
        prefix: str = "sph",
        flush_every: int = 20,
    ) -> None:
        os.makedirs(log_dir, exist_ok=True)
        self._path = os.path.join(log_dir, f"{prefix}_{robot_name}.csv")
        self._file: Optional[TextIO] = open(self._path, "w", newline="")
        self._writer = csv.DictWriter(self._file, fieldnames=self._FIELDNAMES)
        self._writer.writeheader()
        self._t0 = time.time()
        self._flush_every = max(1, flush_every)
        self._rows_since_flush = 0

    def log(self, **kwargs) -> None:
        if self._file is None:
            return
        row = {name: kwargs.get(name, 0.0) for name in self._FIELDNAMES}
        row["t"] = time.time() - self._t0
        row["robot_name"] = kwargs["robot_name"]
        self._writer.writerow(row)
        self._rows_since_flush += 1
        if self._rows_since_flush >= self._flush_every:
            self._file.flush()
            self._rows_since_flush = 0

    def close(self) -> None:
        if self._file is not None:
            self._file.flush()
            self._file.close()
            self._file = None