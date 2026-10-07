"""Simulated-time clock for the SPH controller (ROS-free, so it can be unit-tested).

WHY
    The controller used to tick every 50 ms of WALL time and integrate dt = 50 ms per tick.
    When Gazebo runs at RTF < 1 only RTF*50 ms of simulated time pass between ticks, so the
    SPH model's accelerations act 1/RTF times too strongly on the (simulated) drones.
    With RTF 0.12 (5 drones on your laptop) that is 8x -> instability.

HOW
    Read Gazebo's own clock and tick every `period` SIMULATED seconds:

        GzSimClock   follows `gz topic -e -t <world>/stats` (same source as rtf_logger.py),
                     interpolates between the ~10 Hz wall-time stats messages with the
                     measured RTF, never goes backwards.
        SimTicker    poll(): returns (sim_t, sim_dt) once per `period` of simulated time,
                     otherwise None. If the simulation is paused, no tick happens.

    With this, one tick == `period` s of simulated time at ANY real-time factor, so the
    model integrates exactly the time that passes. The state topic only needs to deliver
    "the latest state"; at low RTF several messages arrive per tick, which is harmless.

    For REAL drones nothing of this is used (sim_clock:=wall): the wall clock IS the
    physical time.
"""

import collections
import os
import re
import shutil
import subprocess
import threading
import time


class StatsParser:
    """Parses the text dump of a gz stats message (`gz topic -e`), line by line.
    feed(line) returns (sim_s, real_s, rtf) when a message is complete, else None."""

    def __init__(self):
        self._cur = None
        self._v = {'sim': [0, 0], 'real': [0, 0]}

    def feed(self, line):
        line = line.strip()
        if line.startswith('sim_time'):
            self._cur = 'sim'
        elif line.startswith('real_time {') or line.startswith('real_time{'):
            self._cur = 'real'
        elif line.startswith('}'):
            self._cur = None
        elif self._cur and line.startswith('sec:'):
            self._v[self._cur][0] = int(line.split(':')[1])
        elif self._cur and line.startswith('nsec:'):
            self._v[self._cur][1] = int(line.split(':')[1])
        elif line.startswith('real_time_factor:'):
            sim = self._v['sim'][0] + self._v['sim'][1] / 1e9
            real = self._v['real'][0] + self._v['real'][1] / 1e9
            rtf = float(line.split(':')[1])
            self._v = {'sim': [0, 0], 'real': [0, 0]}
            return sim, real, rtf
        return None


class WallClock:
    """Placeholder with the same interface: the controller then ticks on wall time."""
    kind = 'wall'
    ready = True
    stalled = False
    rtf = 1.0

    def sim_now(self, wall=None):
        return None

    def close(self):
        pass


class GzSimClock:
    """Simulated time from Gazebo's stats topic (needs the `gz` command, i.e. run it in
    the same container/environment as the simulator).

    argv: full command that prints stats messages to stdout; None = auto-detect
          `gz topic -l` -> world stats topic -> `gz topic -e -t <topic>`.
          (tests pass a fake command.)
    """
    kind = 'gz'

    def __init__(self, topic=None, argv=None, stall_s=2.0, rtf_window=12, max_extrapolate_s=0.5):
        self._topic = topic or None
        self._argv = argv
        self.stall_s = stall_s
        self.max_extrapolate_s = max_extrapolate_s
        self._lock = threading.Lock()
        self._hist = collections.deque(maxlen=rtf_window)   # (wall_monotonic, sim_s)
        self._last_sim = None
        self._last_wall = None
        self._rtf = 1.0
        self._last_ret = None
        self._proc = None
        self._stop = False
        self.msgs = 0
        self.error = None
        self.topic_used = None
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    # ---- reader thread
    def _detect_topic(self):
        t_end = time.monotonic() + 120.0
        while time.monotonic() < t_end and not self._stop:
            try:
                out = subprocess.run(['gz', 'topic', '-l'], capture_output=True, text=True,
                                     timeout=10).stdout.split()
            except (OSError, subprocess.TimeoutExpired):
                out = []
            cands = [t for t in out if t.endswith('/stats')]
            if cands:
                cands.sort(key=lambda t: ('/world/' not in t, len(t)))
                return cands[0]
            time.sleep(1.0)
        return None

    def _run(self):
        try:
            argv = self._argv
            if argv is None:
                topic = self._topic or self._detect_topic()
                if not topic:
                    self.error = 'no */stats topic found (is Gazebo running in this environment?)'
                    return
                self.topic_used = topic
                argv = ['gz', 'topic', '-e', '-t', topic]
                if shutil.which('stdbuf'):
                    argv = ['stdbuf', '-oL'] + argv        # line-buffered, so no 100 ms batches
            self._proc = subprocess.Popen(argv, stdout=subprocess.PIPE, text=True, bufsize=1,
                                          stderr=subprocess.DEVNULL)
            parser = StatsParser()
            for line in self._proc.stdout:
                got = parser.feed(line)
                if got is not None:
                    self._push(*got)
                if self._stop:
                    break
            if not self._stop:
                self.error = 'gz stats stream ended'
        except Exception as ex:                      # noqa: BLE001
            self.error = f'{type(ex).__name__}: {ex}'

    def _push(self, sim, real, rtf_field):
        w = time.monotonic()
        with self._lock:
            if self._last_sim is not None and sim < self._last_sim - 1e-6:
                self._hist.clear()                    # simulation was reset
                self._last_ret = None
            self._hist.append((w, sim))
            self._last_sim, self._last_wall = sim, w
            self.msgs += 1
            if len(self._hist) >= 3:
                (w0, s0), (w1, s1) = self._hist[0], self._hist[-1]
                if w1 - w0 > 0.3:
                    self._rtf = max((s1 - s0) / (w1 - w0), 0.0)
            elif rtf_field and rtf_field > 0:
                self._rtf = rtf_field

    # ---- API
    @property
    def ready(self):
        return self._last_sim is not None

    @property
    def rtf(self):
        return self._rtf

    @property
    def stalled(self):
        """No stats message for stall_s of wall time (sim paused, gz died, ...)."""
        if self._last_wall is None:
            return False
        return time.monotonic() - self._last_wall > self.stall_s

    def sim_now(self, wall=None):
        """Simulated seconds now (monotonic non-decreasing); None until the first message."""
        w = time.monotonic() if wall is None else wall
        with self._lock:
            if self._last_sim is None:
                return None
            ahead = min(max(w - self._last_wall, 0.0), self.max_extrapolate_s)
            s = self._last_sim + ahead * self._rtf
            if self._last_ret is not None and s < self._last_ret:
                s = self._last_ret
            self._last_ret = s
            return s

    def close(self):
        self._stop = True
        p = self._proc
        if p is not None:
            try:
                p.terminate()
            except OSError:
                pass


class SimTicker:
    """One tick per `period` of simulated time.

    poll() -> None (not yet) or (sim_t, sim_dt):
        sim_t   simulated seconds since the first tick
        sim_dt  simulated seconds since the previous tick (about `period`)
    If the loop was starved and several periods have passed, ONE tick is returned and
    `missed` counts the skipped periods (the model then integrates its fixed dt once;
    that is the right thing: the physics did not run faster, the controller was late).
    """

    def __init__(self, clock, period):
        self.clock = clock
        self.period = float(period)
        self._t0 = None
        self._next = None
        self._last = None
        self.missed = 0
        self.ticks = 0

    def poll(self, wall=None):
        s = self.clock.sim_now(wall)
        if s is None:
            return None
        if self._t0 is None:
            self._t0 = s
            self._last = s
            self._next = s + self.period
            return None
        if s < self._next:
            return None
        dt = s - self._last
        late_by = s - self._next
        n_extra = int(late_by // self.period)
        if n_extra:
            self.missed += n_extra
            self._next = s + self.period
        else:
            self._next += self.period
        self._last = s
        self.ticks += 1
        return s - self._t0, dt
