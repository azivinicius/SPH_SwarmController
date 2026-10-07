"""Physics-less particle backend: stands in for CrazySim (firmware + Gazebo physics).

What it does
  * subscribes  /cf_i/cmd_velocity_world   (crazyflie_interfaces/VelocityWorld) -- the SPH controller's output
  * integrates  each particle's velocity toward the command (first-order lag `tau`, optional pure `delay`)
                and its position from the velocity.  No firmware, no rotor model, no collisions.
                Vertical channel (3D runs): own lag `tau_z`, climb limit `max_vz`, a floor at z = 0 and an
                optional constant `z_drift` (m/s) standing in for the gravity / thrust mismatch a real drone
                has: a drone commanded vz = 0 then sinks at z_drift, and only the closed loop (the controller
                reads z back every tick) holds the altitude.
  * publishes   /cf_i/state                (crazyflie_interfaces/LogDataGeneric, values = x y z vx vy vz)
                -- exactly what swarm_sph / plain_sph read from the real drones.
  * shows them  in Gazebo (viz=gz): a world with N visual-only models, moved by calling the
                /world/<world>/set_pose_vector service (UserCommands) at `viz_hz`;
                or in RViz (viz=rviz): one Marker SPHERE_LIST on /particles/markers;
                viz=both | none also exist.  Display never feeds back into the integration.

Because the controller code is unchanged, this answers "does the SPH swarm behave" at a tiny
fraction of the CrazySim cost.  What it can NOT tell you: anything about the real drone (rotor
dynamics, downwash, estimator).  Keep `tau` / `delay` near the values measured on CrazySim at RTF ~ 1
(analyze_run.py check C5 reports them).

Run (dev_tmux.sh up-particles does all of this):
    ros2 run drone_controller particle_sim --ros-args -p total_robots:=5 -p fixed_z:=0.5 \
        -p initial_positions:=0.25,0.0/0.08,0.24/... -p world:=particles -p viz:=gz

The ROS-free ParticleBackend and the pose-message helpers are importable (and tested) without ROS.
"""
import importlib
import math
import shutil
import subprocess
import threading
import time
from collections import deque

import numpy as np

try:                                   # ROS is only needed for the node, not for the backend
    import rclpy
    from rclpy.node import Node
    from crazyflie_interfaces.msg import LogDataGeneric, VelocityWorld
except ImportError:                    # offline tests
    rclpy = None
    Node = object


# --------------------------------------------------------------------------- the model (ROS-free)
class ParticleBackend:
    """N point particles with a first-order velocity loop.  All times are seconds of the SAME clock
    the caller steps with (here: wall time, because the controller also runs on wall time)."""

    def __init__(self, names, positions, z, tau=0.15, delay=0.0, max_speed=0.0,
                 tau_z=None, max_vz=0.0, z_drift=0.0, z_floor=0.0, z_jitter=0.0, seed=1):
        self.names = list(names)
        self.n = len(self.names)
        self.index = {n: i for i, n in enumerate(self.names)}
        pos = np.asarray(positions, dtype=float).reshape(self.n, 2)
        self.x, self.y = pos[:, 0].copy(), pos[:, 1].copy()
        self.z = np.full(self.n, float(z))
        if z_jitter:                      # +-z_jitter m: a perfectly flat start would stay flat (symmetry)
            self.z += np.random.RandomState(seed).uniform(-z_jitter, z_jitter, self.n)
        self.vx = np.zeros(self.n)
        self.vy = np.zeros(self.n)
        self.cx = np.zeros(self.n)      # command in force
        self.cy = np.zeros(self.n)
        self.vz = np.zeros(self.n)      # vertical channel: own lag, own limit, optional drift
        self.cz = np.zeros(self.n)
        self.tau_z = max(float(tau if tau_z is None else tau_z), 1e-6)
        self.max_vz = float(max_vz)
        self.z_drift = float(z_drift)   # m/s added to the vertical velocity: thrust mismatch / battery sag
        self.z_floor = float(z_floor)
        self.tau = max(float(tau), 1e-6)
        self.delay = max(float(delay), 0.0)
        self.max_speed = float(max_speed)
        self.t = 0.0
        self._queue = [deque() for _ in range(self.n)]     # (apply_at, vx, vy), only when delay > 0

    def command(self, name, vx, vy, vz=0.0):
        i = self.index.get(name)
        if i is None:
            return
        if self.max_speed > 0.0:
            sp = math.hypot(vx, vy)
            if sp > self.max_speed:
                vx, vy = vx * self.max_speed / sp, vy * self.max_speed / sp
        if self.max_vz > 0.0:
            vz = max(-self.max_vz, min(self.max_vz, vz))
        if self.delay > 0.0:
            self._queue[i].append((self.t + self.delay, vx, vy, vz))
        else:
            self.cx[i], self.cy[i], self.cz[i] = vx, vy, vz

    def step(self, dt):
        if dt <= 0.0:
            return
        self.t += dt
        if self.delay > 0.0:
            for i, q in enumerate(self._queue):
                while q and q[0][0] <= self.t:
                    _, self.cx[i], self.cy[i], self.cz[i] = q.popleft()
        a = 1.0 - math.exp(-dt / self.tau)          # exact discretisation of dv/dt = (cmd - v) / tau
        self.vx += (self.cx - self.vx) * a
        self.vy += (self.cy - self.vy) * a
        self.x += self.vx * dt
        self.y += self.vy * dt
        az = 1.0 - math.exp(-dt / self.tau_z)
        self.vz += (self.cz + self.z_drift - self.vz) * az
        self.z += self.vz * dt
        low = self.z < self.z_floor                   # the floor: a drone cannot go through it
        if low.any():
            self.z[low] = self.z_floor
            self.vz[low] = np.maximum(self.vz[low], 0.0)

    def state(self, i):
        return [float(self.x[i]), float(self.y[i]), float(self.z[i]), float(self.vx[i]), float(self.vy[i]), float(self.vz[i])]


# --------------------------------------------------------------------------- Gazebo display
def pose_vector_text(names, x, y, z):
    """Protobuf text format of a gz.msgs.Pose_V (identity orientation) for `gz service --req`."""
    return ' '.join(
        f'pose {{ name: "{n}" position {{ x: {xi:.4f} y: {yi:.4f} z: {zi:.4f} }} orientation {{ w: 1.0 }} }}'
        for n, xi, yi, zi in zip(names, x, y, z))


def pose_service_cli(world, text, timeout_ms=500):
    return ['gz', 'service', '-s', f'/world/{world}/set_pose_vector',
            '--reqtype', 'gz.msgs.Pose_V', '--reptype', 'gz.msgs.Boolean',
            '--timeout', str(timeout_ms), '--req', text]


def call_ok(res):
    """True when a gz-transport python `request` result means 'service answered and said true'.
    Bindings return bool, or (bool, response); the response is a gz.msgs.Boolean (.data)."""
    if isinstance(res, tuple):
        ok = bool(res[0])
        rep = res[1] if len(res) > 1 else None
        return ok and (rep is None or getattr(rep, 'data', True) is not False)
    return bool(res)


def cli_ok(proc):
    """`gz service` prints 'data: true' when set_pose_vector worked (stderr/empty/'false' otherwise)."""
    out = (proc.stdout or b'')
    out = out.decode(errors='replace') if isinstance(out, bytes) else str(out)
    return proc.returncode == 0 and 'true' in out.lower()


class GzPoseSender:
    """Moves the world's models.  A background thread keeps only the LATEST pose set (older ones are
    dropped), so a slow Gazebo can never slow the particle loop down.

    Two ways to call the service, tried in this order:
      1. gz-transport python bindings (python3-gz-transport12 + python3-gz-msgs9 for Gazebo Garden):
         persistent node, cheap.  The binding's call signature differs between versions, so a few
         spellings are tried once at start-up.
      2. the `gz service` command line: always works, but every call starts the (ruby) gz launcher,
         so it costs real CPU; keep viz_hz low (<= 5).
    `self.mode` says which one is in use ('bindings' | 'cli' | 'none'); it is logged at start-up."""

    def __init__(self, world, names, log=print, period=0.1, probe=None):
        self.world, self.names, self.log = world, list(names), log
        self._probe = probe                      # (x, y, z) arrays: the real start poses, for a meaningful probe
        self.period = period
        self.mode = 'none'
        self.calls, self.fails, self.last_error = 0, 0, ''
        self.first_ok = False
        self._lock = threading.Lock()
        self._slot = None
        self._stop = False
        self._bind = self._init_bindings()
        if self._bind is None:
            self.mode = 'cli' if shutil.which('gz') else 'none'
        else:
            self.mode = 'bindings'
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _init_bindings(self):
        for tr, ms in (('gz.transport13', 'gz.msgs10'), ('gz.transport12', 'gz.msgs9'),
                       ('gz.transport11', 'gz.msgs8')):
            try:
                Node_ = importlib.import_module(tr).Node
                PoseV = importlib.import_module(ms + '.pose_v_pb2').Pose_V
                Bool = importlib.import_module(ms + '.boolean_pb2').Boolean
            except Exception:
                continue
            node = Node_()
            svc = f'/world/{self.world}/set_pose_vector'
            variants = (
                lambda req: node.request(svc, req, PoseV, Bool, 300),
                lambda req: node.request(svc, req, 300, Bool()),
                lambda req: node.request(svc, req, PoseV, Bool, 300, Bool()),
            )
            probe = PoseV()
            if self._probe is not None:                 # the start poses: harmless, and a real test
                try:
                    from google.protobuf import text_format
                    text_format.Parse(pose_vector_text(self.names, *self._probe), probe)
                except Exception as exc:
                    self.last_error = f'probe: {exc}'
            for call in variants:
                try:
                    res = call(probe)
                except Exception as exc:
                    self.last_error = f'{tr}: {exc}'
                    continue
                if call_ok(res):
                    self.log(f'gz pose sender: python bindings ({tr}), service answered true')
                    return call
                self.last_error = f'{tr}: service did not answer true ({res!r}) - is the world "{self.world}" running?'
                self.log(self.last_error)
        return None

    def submit(self, x, y, z):
        with self._lock:
            self._slot = (np.array(x, dtype=float), np.array(y, dtype=float), np.array(z, dtype=float))

    def _build_request(self, x, y, z):
        pose_v_mod = None
        for ms in ('gz.msgs10', 'gz.msgs9', 'gz.msgs8'):
            try:
                pose_v_mod = importlib.import_module(ms + '.pose_v_pb2')
                break
            except Exception:
                continue
        from google.protobuf import text_format
        req = pose_v_mod.Pose_V()
        text_format.Parse(pose_vector_text(self.names, x, y, z), req)
        return req

    def _run(self):
        while not self._stop:
            time.sleep(self.period)
            with self._lock:
                slot, self._slot = self._slot, None
            if slot is None or self.mode == 'none':
                continue
            x, y, z = slot
            try:
                if self.mode == 'bindings':
                    ok = call_ok(self._bind(self._build_request(x, y, z)))
                    self.last_error = '' if ok else 'service did not answer true'
                else:
                    pr = subprocess.run(pose_service_cli(self.world, pose_vector_text(self.names, x, y, z)),
                                        capture_output=True, timeout=3)
                    ok = cli_ok(pr)
                    self.last_error = '' if ok else ((pr.stderr or pr.stdout or b'').decode(errors='replace')[:160] or 'no answer')
                if ok:
                    self.calls += 1
                    if not self.first_ok:
                        self.first_ok = True
                        self.log('gz pose sender: first pose update accepted by Gazebo')
                else:
                    self.fails += 1
                    if self.fails in (1, 5, 20):
                        self.log(f'gz pose update FAILED ({self.fails}x): {self.last_error}')
            except Exception as exc:
                self.fails += 1
                self.last_error = str(exc)[:200]

    def close(self):
        self._stop = True


# --------------------------------------------------------------------------- the ROS node
def parse_positions(text, n):
    """'x,y/x,y/...' -> list of (x, y); empty -> a ring of radius 0.5."""
    pts = []
    for item in (text or '').split('/'):
        item = item.strip()
        if not item:
            continue
        a, b = item.split(',')
        pts.append((float(a), float(b)))
    if len(pts) != n:
        if text:
            raise ValueError(f'initial_positions has {len(pts)} points but total_robots is {n}')
        pts = [(0.5 * math.cos(2 * math.pi * i / max(n, 1)), 0.5 * math.sin(2 * math.pi * i / max(n, 1)))
               for i in range(n)]
    return pts


class ParticleSimNode(Node):
    def __init__(self):
        super().__init__('particle_sim')
        P = {'total_robots': 3, 'robot_prefix': 'cf_', 'fixed_z': 0.5, 'initial_positions': '',
             'tau': 0.15, 'delay': 0.0, 'max_velocity': 0.0,
             'tau_z': 0.0, 'max_vz': 0.0, 'z_drift': 0.0, 'z_jitter': 0.0, 'state_hz': 20.0, 'max_dt': 0.2,
             'world': 'particles', 'viz': 'gz', 'viz_hz': 10.0, 'frame_id': 'world'}
        for k, v in P.items():
            self.declare_parameter(k, v)
        p = {k: self.get_parameter(k).value for k in P}
        self.n = int(p['total_robots'])
        self.names = [f"{p['robot_prefix']}{i}" for i in range(self.n)]
        self.backend = ParticleBackend(self.names, parse_positions(p['initial_positions'], self.n),
                                       p['fixed_z'], p['tau'], p['delay'], p['max_velocity'],
                                       tau_z=(p['tau_z'] or p['tau']), max_vz=p['max_vz'], z_drift=p['z_drift'],
                                       z_jitter=p['z_jitter'])
        self.p = p
        self._pubs = {nm: self.create_publisher(LogDataGeneric, f'/{nm}/state', 10) for nm in self.names}
        for nm in self.names:
            self.create_subscription(VelocityWorld, f'/{nm}/cmd_velocity_world',
                                     lambda msg, n=nm: self._cmd(n, msg), 10)
        viz = str(p['viz'])
        self.sender = None
        self.marker_pub = None
        if viz in ('gz', 'both'):
            b0 = self.backend
            self.sender = GzPoseSender(p['world'], self.names, log=lambda s: self.get_logger().info(s),
                                       period=1.0 / max(float(p['viz_hz']), 0.5),
                                       probe=(b0.x.copy(), b0.y.copy(), b0.z.copy()))
            self.get_logger().info(f'Gazebo display: world "{p["world"]}", mode {self.sender.mode}, {p["viz_hz"]} Hz')
            if self.sender.mode == 'none':
                self.get_logger().warn('no gz command and no python bindings: nothing will move in Gazebo')
        if viz in ('rviz', 'both'):
            from visualization_msgs.msg import Marker
            self._Marker = Marker
            self.marker_pub = self.create_publisher(Marker, '/particles/markers', 1)
        self._t_last = time.monotonic()
        self._t_viz = 0.0
        self._cost = deque(maxlen=200)
        self._t_report = time.monotonic()
        self._n_cmd = 0
        self.create_timer(1.0 / float(p['state_hz']), self._tick)
        self.get_logger().info(
            f'particle_sim: {self.n} particles, tau={p["tau"]} s, delay={p["delay"]} s, z={p["fixed_z"]} m, '
            f'state {p["state_hz"]} Hz, viz={viz}')

    def _cmd(self, name, msg):
        self.backend.command(name, float(msg.vel.x), float(msg.vel.y), float(msg.vel.z))
        self._n_cmd += 1

    def _tick(self):
        t0 = time.monotonic()
        dt = min(max(t0 - self._t_last, 0.0), float(self.p['max_dt']))
        self._t_last = t0
        b = self.backend
        b.step(dt)
        stamp = self.get_clock().now().to_msg()
        for i, nm in enumerate(self.names):
            m = LogDataGeneric()
            try:
                m.header.stamp = stamp
            except Exception:
                pass
            m.values = b.state(i)
            self._pubs[nm].publish(m)
        if (self.sender is not None or self.marker_pub is not None) and \
                t0 - self._t_viz >= 1.0 / max(float(self.p['viz_hz']), 0.5):
            self._t_viz = t0
            if self.sender is not None:
                self.sender.submit(b.x, b.y, b.z)
            if self.marker_pub is not None:
                self._publish_markers(stamp)
        self._cost.append((time.monotonic() - t0) * 1000.0)
        if t0 - self._t_report >= 10.0:
            self._t_report = t0
            extra = ''
            if self.sender is not None:
                extra = f', gz calls {self.sender.calls} (failed {self.sender.fails}) {self.sender.last_error}'
            self.get_logger().info(
                f'particle_sim: step {np.mean(self._cost):.2f} ms mean / {max(self._cost):.2f} max, '
                f'commands received {self._n_cmd}{extra}')

    def _publish_markers(self, stamp):
        from geometry_msgs.msg import Point
        mk = self._Marker()
        mk.header.frame_id = str(self.p['frame_id'])
        mk.header.stamp = stamp
        mk.ns, mk.id = 'particles', 0
        mk.type, mk.action = self._Marker.SPHERE_LIST, self._Marker.ADD
        mk.scale.x = mk.scale.y = mk.scale.z = 0.1
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = 0.1, 0.5, 1.0, 1.0
        mk.pose.orientation.w = 1.0
        b = self.backend
        mk.points = [Point(x=float(b.x[i]), y=float(b.y[i]), z=float(b.z[i])) for i in range(self.n)]
        self.marker_pub.publish(mk)

    def close(self):
        if self.sender is not None:
            self.sender.close()


def main(args=None):
    rclpy.init(args=args)
    node = ParticleSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()