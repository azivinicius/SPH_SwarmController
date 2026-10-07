"""ROS glue shared by plain_sph.py and swarm_sph.py: ROS parameters -> SphCore.

Everything that is not ROS lives in sph_core.py / sph_diag.py so it can run offline.
"""

import time

from rclpy.time import Time

from drone_controller import sph_core
from drone_controller.sim_clock import GzSimClock, SimTicker, WallClock
from drone_controller.sph_core import COMMON_DEFAULTS, LOG_DIR  # noqa: F401  (re-exported)


def read_params(node, overrides=None):
    defaults = dict(COMMON_DEFAULTS)
    defaults.update(overrides or {})
    cfg = {}
    for key, default in defaults.items():
        node.declare_parameter(key, default)
        cfg[key] = node.get_parameter(key).value
    return cfg


def build_core(node, cfg, egos_from_names, density_on_rx, tag, write_meta=True):
    def echo(level, text):
        lg = node.get_logger()
        (lg.error if level == 'ERROR' else lg.warn)(text)

    return sph_core.build_core(cfg, egos_from_names, density_on_rx, tag,
                               echo=echo, write_meta=write_meta)


def header_latency_ms(node, msg):
    """now - msg.header.stamp in ms, or None if the message has no usable stamp."""
    try:
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            return None
        return (node.get_clock().now() - Time.from_msg(stamp)).nanoseconds / 1e6
    except Exception:
        return None


class ControlDriver:
    """Owns the control timer of a node.

    sim_clock=wall : timer(control_period) -> core.step(now)           (old behaviour)
    sim_clock=gz   : timer(poll_period)    -> one core.step(now, sim_t, sim_dt) per
                     control_period of SIMULATED time; the last commands are re-sent every
                     keepalive_s of wall time so the firmware's setpoint watchdog stays
                     happy while the simulation runs slowly.

    publish({drone: (vx, vy)}) is supplied by the node.
    """

    def __init__(self, node, cfg, core, now_fn, publish):
        self.node, self.core, self.now, self.publish = node, core, now_fn, publish
        self.mode = cfg['sim_clock']
        self.period = float(cfg['control_period'])
        self.keepalive = float(cfg['keepalive_s'])
        self._last_cmds, self._last_pub = {}, 0.0
        self._w0 = time.monotonic()
        self._warned_missing = False
        self._stalled = False
        self._health_next = 0.0
        if self.mode == 'gz':
            self.clock = GzSimClock(topic=cfg['gz_stats_topic'] or None)
            self.ticker = SimTicker(self.clock, self.period)
            node.create_timer(float(cfg['poll_period']), self._poll)
            node.get_logger().info(
                f'sim_clock=gz: ticking every {self.period * 1e3:.0f} ms of SIMULATED time '
                f'(waiting for the Gazebo stats topic)')
        elif self.mode == 'wall':
            self.clock = WallClock()
            node.create_timer(self.period, self._wall_tick)
        else:
            raise ValueError("sim_clock must be 'wall' or 'gz'")

    def _send(self, out):
        self.publish(out)
        self._last_pub = time.monotonic()

    def _wall_tick(self):
        self._send(self.core.step(self.now()))

    def _poll(self):
        w = time.monotonic()
        tk = self.ticker.poll(w)
        if tk is not None:
            out = self.core.step(self.now(), tk[0], tk[1])
            self._last_cmds.update({n: c for n, c in out.items() if c is not None})
            self._send(out)
        elif self._last_cmds and w - self._last_pub >= self.keepalive:
            self._send(self._last_cmds)          # keepalive: same command, no integration
        if w >= self._health_next:
            self._health_next = w + 1.0
            self._health(w)

    def _health(self, w):
        now = self.now()
        log = self.core.log
        if not self.clock.ready:
            if w - self._w0 > 15.0 and not self._warned_missing:
                self._warned_missing = True
                why = self.clock.error or 'no message yet'
                log.event('ERROR', 'SIMCLOCK_MISSING',
                          f'no Gazebo stats after 15 s ({why}); the controller sends NOTHING until the '
                          f'simulated clock appears. Is gz in this environment? try: gz topic -l | grep stats',
                          t=now)
            return
        if self.clock.stalled and not self._stalled:
            self._stalled = True
            log.event('WARN', 'SIMCLOCK_STALL',
                      'no Gazebo stats for 2 s: simulation paused or gz stream died; ticks pause too', t=now)
        elif not self.clock.stalled and self._stalled:
            self._stalled = False
            log.event('INFO', 'SIMCLOCK_RESUME', 'Gazebo clock running again', t=now)

    def close(self):
        if self.mode == 'gz':
            self.core.log.event('INFO', 'SIMCLOCK_STOP',
                                f'ticks={self.ticker.ticks} missed_periods={self.ticker.missed} '
                                f'rtf_last={self.clock.rtf:.3f}', t=self.now())
        self.clock.close()
