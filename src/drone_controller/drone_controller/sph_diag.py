"""Run diagnostics for the SPH controllers -- ROS-free on purpose.

Nothing in here imports rclpy, so the same code is exercised offline by
src/offline_twin.py and by the unit checks, and in the sim by plain_sph.py /
swarm_sph.py (through sph_core.SphCore).

One run == one folder:  <log_dir>/runs/<run_id>/
    meta.json            every parameter of the run + code version (written once)
    tick_<drone>.csv     one row per CONTROL TICK per drone      (TICK_COLUMNS)
    rx_<drone>.csv       one row per STATE MESSAGE received      (RX_COLUMNS)
    events.log           human-readable timeline: startup, warnings, heartbeats
    rtf.csv              Gazebo real-time factor (written by tools/rtf_logger.py)
    analysis/            written by src/analyze_run.py  (report.md + figures)

Legacy file logs/plain_sph_<drone>.csv (old 9 columns) is still written so
plot_plain_sph.py keeps working.
"""

import csv
import json
import math
import os
import time

# --------------------------------------------------------------------------- columns
TICK_COLUMNS = [
    # --- when ---------------------------------------------------------------
    't',            # s since the node started (ROS clock, i.e. wall clock here)
    'epoch',        # unix time (s); lets the analyzer line up rtf.csv and other logs
    'tick',         # control-tick counter of this node
    'drone',
    'wall_dt_ms',   # time since the previous tick  (nominal = 1000*period; jitter / stalls)
    'step_ms',      # cost of the whole control step (all egos of this node)
    'sim_t',        # simulated seconds (sim_clock=gz), blank on wall-clock runs
    'sim_dt_ms',    # simulated time since the previous tick (sim_clock=gz)
    'dt_used_ms',   # dt the SPH model integrated this tick
    # --- what the controller did ---------------------------------------------
    'gate',         # 1 = start gate open (controller allowed to act)
    'src',          # command source: sph | sph_pred | hold | skip | gate | fence | test:<profile>
    # --- freshness of the information it used -----------------------------------
    'rx_since',     # state messages of THIS drone received since the previous tick (0 = stale tick)
    'state_age_ms', # age of this drone's last state message at tick time
    'stale_run',    # consecutive stale ticks so far (0 if this tick is fresh)
    'n_known',      # drones whose state has been received at least once
    'n_nb_fresh',   # neighbours whose state is younger than ~3 control periods
    'max_nb_age_ms',
    # --- measured by the drone (last message) ---------------------------------
    'meas_x', 'meas_y', 'meas_z', 'meas_vx', 'meas_vy', 'meas_vz',
    # --- state SPH actually used (== measured on fresh ticks, PREDICTED on stale ones)
    'used_x', 'used_y', 'used_vx', 'used_vy',
    'pred_err',     # |used position - measured position| (m)
    # --- SPH state -------------------------------------------------------------
    'rho', 'P', 'c', 'n_nb', 'nn_dist', 'mean_nb_dist',
    # --- acceleration budget (m/s^2): the five parts of dv/dt --------------------
    'ap_x', 'ap_y',   # pressure term
    'av_x', 'av_y',   # artificial viscosity term
    'ad_x', 'ad_y',   # extra damping  -zetta_extra * v
    'ae_x', 'ae_y',   # external potential force
    'a_x', 'a_y',     # total dv/dt (after sph_gain)
    # --- command --------------------------------------------------------------
    'cmd_raw_x', 'cmd_raw_y',   # velocity before the speed clip
    'cmd_x', 'cmd_y',           # velocity actually published (after clip)
    'clipped',                  # 1 if the clip changed it
    'prev_cmd_x', 'prev_cmd_y', # command published on the previous tick
    # --- geometry --------------------------------------------------------------
    'cen_x', 'cen_y',   # centroid of the known drones (controller's view)
    'r_cen',            # |drone - centroid|
    'r_pot',            # |drone - potential centre (xc, yc)|   (3D mode: xc, yc, zc)
    # --- 3D mode only (blank in 2D runs) ---------------------------------------
    'used_z', 'used_vz', 'ap_z', 'av_z', 'ad_z', 'ae_z', 'a_z',
    'cmd_raw_z', 'cmd_z', 'prev_cmd_z', 'cen_z',
]

RX_COLUMNS = [
    't', 'epoch', 'drone',
    'rx_dt_ms',        # time since the previous message from this drone
    'x', 'y', 'z', 'vx', 'vy', 'vz',
    'hdr_lat_ms',      # now - msg.header.stamp, if the message carries a stamp (else blank)
]

LEGACY_COLUMNS = ['t', 'rho', 'pressure', 'ext_force_x', 'ext_force_y',
                  'vel_x', 'vel_y', 'x', 'y']

_INT_COLS = {'tick', 'gate', 'rx_since', 'stale_run', 'n_known', 'n_nb_fresh',
             'n_nb', 'clipped'}
_STR_COLS = {'drone', 'src'}


def _fmt(col, v):
    if v is None:
        return ''
    if col in _STR_COLS:
        return v
    if col in _INT_COLS:
        return int(v)
    if col == 't':
        return f'{v:.4f}'
    if col == 'epoch':
        return f'{v:.3f}'
    try:
        if isinstance(v, float) and math.isnan(v):
            return 'nan'
        return f'{float(v):.7g}'
    except (TypeError, ValueError):
        return v


# --------------------------------------------------------------------------- logger
class RunLogger:
    """Writes the run folder. All file errors are swallowed after the first
    warning: a logging problem must never stop the controller."""

    def __init__(self, log_dir, run_id=None, tag='node', echo=None, flush_every=20):
        self.log_dir = log_dir
        self.run_id = run_id or time.strftime('%Y%m%d_%H%M%S')
        self.tag = tag
        self.echo = echo                       # callable(level, text) -> mirrors events to the ROS log
        self._flush_every = max(1, flush_every)
        self.run_dir = os.path.join(log_dir, 'runs', self.run_id)
        self._tick = {}
        self._rx = {}
        self._legacy = {}
        self._n_since_flush = 0
        self._broken = False
        self.counts = {'INFO': 0, 'WARN': 0, 'ERROR': 0}
        try:
            os.makedirs(self.run_dir, exist_ok=True)
            self._events = open(os.path.join(self.run_dir, 'events.log'), 'a', buffering=1)
        except OSError as exc:
            self._events = None
            self._broken = True
            if self.echo:
                self.echo('WARN', f'run logger disabled, cannot write {self.run_dir}: {exc}')

    # ------------------------------------------------------------------ files
    def write_meta(self, meta, overwrite=False):
        """meta.json is written by ONE process per run (the first to get here)."""
        if self._broken:
            return
        path = os.path.join(self.run_dir, 'meta.json')
        if os.path.exists(path) and not overwrite:
            return
        try:
            with open(path, 'w') as f:
                json.dump(meta, f, indent=2, sort_keys=True, default=str)
            latest = os.path.join(self.log_dir, 'runs', 'latest')
            try:
                if os.path.islink(latest) or os.path.exists(latest):
                    os.remove(latest)
                os.symlink(self.run_id, latest)
            except OSError:
                pass
        except OSError as exc:
            self._warn_once(f'cannot write meta.json: {exc}')

    def open_drone(self, name, legacy_dir=None):
        if self._broken:
            return
        try:
            f = open(os.path.join(self.run_dir, f'tick_{name}.csv'), 'w', newline='')
            w = csv.writer(f)
            w.writerow(TICK_COLUMNS)
            self._tick[name] = (f, w)
            f = open(os.path.join(self.run_dir, f'rx_{name}.csv'), 'w', newline='')
            w = csv.writer(f)
            w.writerow(RX_COLUMNS)
            self._rx[name] = (f, w)
            if legacy_dir:
                os.makedirs(legacy_dir, exist_ok=True)
                f = open(os.path.join(legacy_dir, f'plain_sph_{name}.csv'), 'w', newline='')
                w = csv.writer(f)
                w.writerow(LEGACY_COLUMNS)
                self._legacy[name] = (f, w)
        except OSError as exc:
            self._warn_once(f'cannot open logs for {name}: {exc}')

    def tick(self, name, row, legacy=None):
        w = self._tick.get(name)
        if w is None:
            return
        try:
            w[1].writerow([_fmt(c, row.get(c)) for c in TICK_COLUMNS])
            lw = self._legacy.get(name)
            if lw is not None and legacy is not None:
                lw[1].writerow(legacy)
            self._count_flush()
        except (OSError, ValueError) as exc:
            self._warn_once(f'tick write failed: {exc}')

    def rx(self, name, row):
        w = self._rx.get(name)
        if w is None:
            return
        try:
            w[1].writerow([_fmt(c, row.get(c)) for c in RX_COLUMNS])
            self._count_flush()
        except (OSError, ValueError) as exc:
            self._warn_once(f'rx write failed: {exc}')

    def event(self, level, code, msg, t=None, name=None):
        """One line in events.log (+ echoed to the ROS log for WARN/ERROR)."""
        self.counts[level] = self.counts.get(level, 0) + 1
        stamp = f'{t:9.3f}' if t is not None else '    -    '
        who = (name or self.tag).ljust(7)
        line = f'{stamp} {level:<5} {who} {code:<12} {msg}'
        if self._events is not None:
            try:
                self._events.write(line + '\n')
            except OSError:
                pass
        if self.echo is not None and level != 'INFO':
            self.echo(level, f'{code} {msg}')
        if level != 'INFO':
            self.flush()

    def flush(self):
        for group in (self._tick, self._rx, self._legacy):
            for f, _w in group.values():
                try:
                    f.flush()
                except (OSError, ValueError):
                    pass

    def close(self):
        self.flush()
        for group in (self._tick, self._rx, self._legacy):
            for f, _w in group.values():
                try:
                    f.close()
                except (OSError, ValueError):
                    pass
            group.clear()
        if self._events is not None:
            try:
                self._events.close()
            except OSError:
                pass
            self._events = None

    # ------------------------------------------------------------------ internals
    def _count_flush(self):
        self._n_since_flush += 1
        if self._n_since_flush >= self._flush_every:
            self.flush()
            self._n_since_flush = 0

    def _warn_once(self, text):
        if not self._broken and self.echo:
            self.echo('WARN', text)
        self._broken = True


# --------------------------------------------------------------------------- test commands
PROFILE_KINDS = ('none', 'zero', 'step', 'square', 'sine', 'chirp')


class CommandProfile:
    """Known velocity commands to send INSTEAD of the SPH command, to measure the
    drone's own response (gain / lag / drift) without SPH in the loop.

        none    SPH drives the drones (normal run)
        zero    command 0 m/s: pure hover -> measures drift of the drone itself
        step    0 for `delay` s, then +amp for `period` s, then 0
        square  +amp / -amp alternating, full cycle = `period` s
        sine    amp * sin(2 pi t / period)
        chirp   sine whose frequency sweeps 0.05 -> 1.5 Hz (log) over `duration` s

    The command is the same for every drone (the swarm translates as a block), along
    `axis` = x or y.  Time zero is when the start gate opens (see SphCore).
    """

    def __init__(self, kind='none', amp=0.15, period=4.0, axis='x', delay=3.0, duration=60.0):
        if kind not in PROFILE_KINDS:
            raise ValueError(f'test_profile must be one of {PROFILE_KINDS}, got {kind!r}')
        if axis not in ('x', 'y'):
            raise ValueError("test_axis must be 'x' or 'y'")
        self.kind, self.amp, self.period = kind, float(amp), max(float(period), 1e-3)
        self.axis, self.delay, self.duration = axis, float(delay), max(float(duration), 1.0)
        self.f0, self.f1 = 0.05, 1.5

    @property
    def active(self):
        return self.kind != 'none'

    def total_time(self):
        """Seconds (after the gate opens) until the profile has finished."""
        if self.kind == 'zero':
            return self.delay + 30.0
        if self.kind == 'step':
            return self.delay + self.period + 5.0
        if self.kind == 'square':
            return self.delay + 4 * self.period
        if self.kind == 'sine':
            return self.delay + 4 * self.period
        if self.kind == 'chirp':
            return self.delay + self.duration
        return 0.0

    def scalar(self, s):
        """Command along the axis, s = seconds since the gate opened."""
        if self.kind in ('none', 'zero') or s < self.delay:
            return 0.0
        u = s - self.delay
        if self.kind == 'step':
            return self.amp if u < self.period else 0.0
        if self.kind == 'square':
            return self.amp if (u % self.period) < self.period / 2 else -self.amp
        if self.kind == 'sine':
            return self.amp * math.sin(2 * math.pi * u / self.period)
        if self.kind == 'chirp':
            if u > self.duration:
                return 0.0
            k = math.log(self.f1 / self.f0) / self.duration
            phase = 2 * math.pi * self.f0 * (math.exp(k * u) - 1.0) / k
            return self.amp * math.sin(phase)
        return 0.0

    def value(self, s):
        v = self.scalar(s)
        return (v, 0.0) if self.axis == 'x' else (0.0, v)

    def describe(self):
        return (f'{self.kind} amp={self.amp} m/s period={self.period} s axis={self.axis} '
                f'delay={self.delay} s duration={self.duration} s')


# --------------------------------------------------------------------------- small helpers
def clip_norm(vx, vy, max_norm):
    """Same rule as Commander._clip_norm, on plain floats."""
    if max_norm is None:
        return vx, vy
    n = math.hypot(vx, vy)
    if n <= max_norm or n == 0.0:
        return vx, vy
    s = max_norm / n
    return vx * s, vy * s


def git_info(path):
    """Short commit hash (+'-dirty') of the repo containing `path`, or 'unknown'."""
    try:
        import subprocess
        out = subprocess.run(['git', '-C', path, 'rev-parse', '--short', 'HEAD'],
                             capture_output=True, text=True, timeout=3)
        if out.returncode != 0:
            return 'unknown'
        dirty = subprocess.run(['git', '-C', path, 'status', '--porcelain'],
                               capture_output=True, text=True, timeout=3).stdout.strip()
        return out.stdout.strip() + ('-dirty' if dirty else '')
    except Exception:
        return 'unknown'