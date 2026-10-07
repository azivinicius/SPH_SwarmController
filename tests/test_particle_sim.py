#!/usr/bin/env python3
"""Offline self-checks of the physics-less particle backend:  python3 tests/test_particle_sim.py

No ROS, no Gazebo.  The node is exercised with stub rclpy / crazyflie_interfaces modules; the
Gazebo-facing calls are only checked for FORMAT (they cannot be run here)."""
import math
import os
import sys
import types
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, 'src', 'drone_controller'))
sys.path.insert(0, ROOT)

FAILS = []


def check(name, ok, info=''):
    print(('PASS ' if ok else 'FAIL ') + name + (f'  [{info}]' if info else ''))
    if not ok:
        FAILS.append(name)


from drone_controller import particle_sim as ps   # noqa: E402

# --- backend: first-order response, exact for any dt ---------------------------------------------
b = ps.ParticleBackend(['cf_0', 'cf_1'], [(0.0, 0.0), (1.0, 0.0)], 0.5, tau=0.15)
b.command('cf_0', 0.4, 0.0)
for _ in range(100):
    b.step(0.01)                       # 1 s
vx_exact = 0.4 * (1 - math.exp(-1.0 / 0.15))
check('step response matches 1-exp(-t/tau)', abs(b.vx[0] - vx_exact) < 1e-9, f'{b.vx[0]:.5f} vs {vx_exact:.5f}')
check('other particle untouched', b.vx[1] == 0.0 and abs(b.x[1] - 1.0) < 1e-12)
b2 = ps.ParticleBackend(['a'], [(0, 0)], 0.5, tau=0.15)
b2.command('a', 0.4, 0.0)
for _ in range(10):
    b2.step(0.1)
check('result independent of dt (exact discretisation)', abs(b2.vx[0] - vx_exact) < 1e-9)

# --- delay -----------------------------------------------------------------------------------------
d = ps.ParticleBackend(['a'], [(0, 0)], 0.5, tau=0.01, delay=0.3)
d.command('a', 1.0, 0.0)
for _ in range(20):
    d.step(0.01)                       # 0.2 s < delay
check('command waits for the delay', abs(d.vx[0]) < 1e-9)
for _ in range(30):
    d.step(0.01)                       # 0.5 s
check('command applied after the delay', d.vx[0] > 0.9)

# --- speed clip ------------------------------------------------------------------------------------
c = ps.ParticleBackend(['a'], [(0, 0)], 0.5, tau=0.01, max_speed=0.5)
c.command('a', 3.0, 4.0)
for _ in range(200):
    c.step(0.01)
check('speed clipped, direction kept', abs(math.hypot(c.vx[0], c.vy[0]) - 0.5) < 1e-3 and abs(c.vy[0] / c.vx[0] - 4 / 3) < 1e-3)
check('unknown name ignored', c.command('nope', 1, 1) is None)
check('state layout x y z vx vy vz', len(b.state(0)) == 6 and b.state(0)[2] == 0.5 and b.state(0)[5] == 0.0)

# --- gz messages -----------------------------------------------------------------------------------
txt = ps.pose_vector_text(['cf_0', 'cf_1'], [0.1, 0.2], [0.3, 0.4], [0.5, 0.5])
check('pose text has both names', txt.count('pose {') == 2 and 'name: "cf_1"' in txt and 'x: 0.2000' in txt)
cli = ps.pose_service_cli('particles', txt)
check('cli targets set_pose_vector', cli[:4] == ['gz', 'service', '-s', '/world/particles/set_pose_vector']
      and 'gz.msgs.Pose_V' in cli and cli[-1] == txt)
try:
    from google.protobuf import text_format   # noqa: F401
    have_pb = True
except ImportError:
    have_pb = False
print('INFO protobuf available:', have_pb)

class _R:
    def __init__(self, data):
        self.data = data


check('call_ok bool / tuple', ps.call_ok(True) and not ps.call_ok(False) and ps.call_ok((True, _R(True)))
      and not ps.call_ok((True, _R(False))) and not ps.call_ok((False, _R(True))))
P = types.SimpleNamespace
check('cli_ok needs data: true', ps.cli_ok(P(returncode=0, stdout=b'data: true\n', stderr=b''))
      and not ps.cli_ok(P(returncode=0, stdout=b'data: false\n', stderr=b''))
      and not ps.cli_ok(P(returncode=1, stdout=b'', stderr=b'timeout')))

# --- parse_positions -------------------------------------------------------------------------------
check('parse_positions', ps.parse_positions('0.25,0.0/-0.1,0.2', 2) == [(0.25, 0.0), (-0.1, 0.2)])
check('parse_positions default ring', len(ps.parse_positions('', 4)) == 4)
try:
    ps.parse_positions('1,2', 3)
    check('parse_positions count mismatch raises', False)
except ValueError:
    check('parse_positions count mismatch raises', True)

# --- world SDF -------------------------------------------------------------------------------------
import gen_config   # noqa: E402
pos = gen_config.positions(5, 0.255)
root = ET.fromstring(gen_config.world_sdf(pos, 0.5, 0.255))
world = root.find('world')
models = [m.get('name') for m in world.findall('model')]
check('sdf world named particles', world.get('name') == 'particles')
check('sdf has cf_0..cf_4 + ground', models == ['ground'] + [f'cf_{i}' for i in range(5)], str(models))
check('sdf zero gravity + Physics + UserCommands',
      world.find('gravity').text.split() == ['0', '0', '0']
      and {p.get('name') for p in world.findall('plugin')} >= {'gz::sim::systems::Physics', 'gz::sim::systems::UserCommands'})
check('sdf particles have no collision', world.find(".//model[@name='cf_0']//collision") is None)

# --- grid layout -------------------------------------------------------------------------------------
import gen_config as _g   # noqa: E402
for n in (1, 3, 5, 9):
    g = _g.grid_positions(n, 0.3)
    dmin = min((math.hypot(a[0] - b[0], a[1] - b[1]) for i, a in enumerate(g) for b in g[i + 1:]), default=0.3)
    check(f'grid n={n}: {len(g)} points, spacing >= 0.3, centred in x',
          len(g) == n and dmin >= 0.3 - 1e-9 and (n == 1 or dmin < 0.34) and abs(sum(p[0] for p in g)) < 1e-9)

# --- node wiring with stubs --------------------------------------------------------------------------
sent, published = [], []


class _Stamp:
    def to_msg(self):
        return 0


class _Node:
    def __init__(self, name):
        self.params, self.timers, self.subs = {}, [], {}

    def declare_parameter(self, k, v):
        self.params[k] = v

    def get_parameter(self, k):
        return types.SimpleNamespace(value=self.params[k])

    def create_publisher(self, typ, topic, q):
        return types.SimpleNamespace(publish=lambda m, t=topic: published.append((t, list(m.values))))

    def create_subscription(self, typ, topic, cb, q):
        self.subs[topic] = cb

    def create_timer(self, period, cb):
        self.timers.append(cb)

    def get_logger(self):
        return types.SimpleNamespace(info=lambda *a: None, warn=lambda *a: None)

    def get_clock(self):
        return types.SimpleNamespace(now=lambda: _Stamp())


class _Msg:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None)
        self.values = []
        self.vel = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)


over = {'total_robots': 2, 'viz': 'none', 'initial_positions': '0.0,0.0/1.0,0.0', 'tau': 0.05}
_decl = _Node.declare_parameter
_Node.declare_parameter = lambda self, k, v: _decl(self, k, over.get(k, v))
for name, attrs in (('rclpy', {}), ('rclpy.node', {'Node': _Node}),
                    ('crazyflie_interfaces', {}),
                    ('crazyflie_interfaces.msg', {'LogDataGeneric': _Msg, 'VelocityWorld': _Msg})):
    m = types.ModuleType(name)
    m.__dict__.update(attrs)
    sys.modules[name] = m
import importlib   # noqa: E402
ps = importlib.reload(ps)
node = ps.ParticleSimNode()
check('node subscribes to both cmd topics', set(node.subs) == {'/cf_0/cmd_velocity_world', '/cf_1/cmd_velocity_world'})
msg = _Msg()
msg.vel.x = 0.3
node.subs['/cf_0/cmd_velocity_world'](msg)
import time as _t   # noqa: E402
for _ in range(5):
    _t.sleep(0.05)
    node.timers[0]()
st = [v for t, v in published if t == '/cf_0/state'][-1]
check('node publishes /cf_0/state with moving x', st[0] > 0.0 and st[3] > 0.0 and len(st) == 6, str(st))
st1 = [v for t, v in published if t == '/cf_1/state'][-1]
check('node keeps cf_1 still', abs(st1[0] - 1.0) < 1e-9 and st1[3] == 0.0)

print('\nALL PASS' if not FAILS else '\nFAILED: ' + ', '.join(FAILS))
sys.exit(1 if FAILS else 0) 