#!/usr/bin/env python3
"""Offline digital twin: the REAL control loop (sph_core.SphCore) against a toy drone.

No ROS, no Gazebo, no Docker -- runs on any machine with python3 + numpy:

    python3 offline_twin.py --n 3 --seconds 60 --rtf 0.4 --label twin_rtf04
    python3 analyze_run.py twin_logs/runs/*twin_rtf04

Why it exists
  1. It writes exactly the same run folder as the real nodes, so the analyzer (and you)
     can be checked on data where the cause is KNOWN.
  2. It is the reference "ideal drone" experiment: if the twin with a plain first-order
     drone stays calm and your Gazebo run shakes, the SPH parameters are not the
     problem -- the simulator / firmware / timing is.
  3. It lets you turn ONE suspect at a time (sim speed, state rate, lag, delay, drift
     bias) and see its fingerprint in the report, before spending 3 minutes of
     Gazebo per experiment.

What the toy drone does (all of it in SIMULATED time, which runs at --rtf x wall time)
  * horizontal velocity follows the command with a first-order lag --tau and a pure
    delay --delay; --bias adds an outward drift (m/s) on the drones in --bias-drones
  * on the ground (before --takeoff-at) the drone does not move horizontally,
    then climbs to z = fixed_z in 2 s
  * the drone publishes its state every 1/--state-hz SIMULATED seconds, i.e. every
    1/(state-hz*rtf) WALL seconds -- the same mechanism as the 20 Hz state topic
    of a real-time-factor-0.4 Gazebo
  * the controller timer runs at wall-clock 20 Hz (+ jitter, + rare stalls)
It is a model, not Gazebo: use it to understand the instruments and the mechanisms,
then confirm on the real simulator.
"""
import argparse
import heapq
import math
import os
import random
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, 'drone_controller'))   # the ROS package dir
sys.path.insert(0, os.path.dirname(HERE))          # project root (gen_config.py)

from drone_controller import sph_core            # noqa: E402


class SimDrone:
    def __init__(self, name, x, y, tau, delay, bias, z_target, rng):
        self.name, self.x, self.y = name, x, y
        self.z, self.vx, self.vy = 0.0, 0.0, 0.0
        self.tau, self.delay, self.bias = tau, delay, bias
        self.z_target = z_target
        self.cmd_queue = []            # (apply_at_sim_time, vx, vy)
        self.cmd = (0.0, 0.0)
        self.flying = False
        self.rng = rng
        self.next_pub_sim = 0.0

    def command(self, sim_t, vx, vy):
        self.cmd_queue.append((sim_t + self.delay, vx, vy))

    def advance(self, sim_t, dt):
        while self.cmd_queue and self.cmd_queue[0][0] <= sim_t:
            _, vx, vy = self.cmd_queue.pop(0)
            self.cmd = (vx, vy)
        if self.flying and self.z < self.z_target:
            self.z = min(self.z_target, self.z + 0.25 * dt)
        if self.z > 0.05:
            tgt_x, tgt_y = self.cmd[0] + self.bias[0], self.cmd[1] + self.bias[1]
            a = min(1.0, dt / max(self.tau, 1e-6))
            self.vx += (tgt_x - self.vx) * a
            self.vy += (tgt_y - self.vy) * a
            self.x += self.vx * dt
            self.y += self.vy * dt
        else:
            self.vx = self.vy = 0.0

    def state(self, sx, sv):
        r = self.rng.gauss
        return [self.x + r(0, sx), self.y + r(0, sx), self.z + r(0, sx),
                self.vx + r(0, sv), self.vy + r(0, sv), r(0, sv)]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    # --- experiment -----------------------------------------------------------
    ap.add_argument('--n', type=int, default=3)
    ap.add_argument('--seconds', type=float, default=60.0, help='WALL seconds to simulate')
    ap.add_argument('--label', default='twin')
    ap.add_argument('--out', default=os.path.join(HERE, 'twin_logs'))
    ap.add_argument('--mode', choices=['swarm', 'plain'], default='swarm',
                    help='swarm = one node for all drones; plain = one node per drone')
    ap.add_argument('--seed', type=int, default=1)
    # --- toy drone / simulator --------------------------------------------------
    ap.add_argument('--rtf', type=float, default=1.0, help='real-time factor of the simulator')
    ap.add_argument('--rtf-sigma', type=float, default=0.0, help='random-walk wobble of the RTF')
    ap.add_argument('--tau', type=float, default=0.15, help='velocity-loop lag of the drone, sim s')
    ap.add_argument('--delay', type=float, default=0.10, help='command delay of the drone, sim s')
    ap.add_argument('--state-hz', type=float, default=20.0, help='state topic rate in SIM time')
    ap.add_argument('--rx-latency', type=float, default=0.01, help='state message latency, wall s')
    ap.add_argument('--bias', type=float, default=0.0, help='outward drift speed (m/s) ...')
    ap.add_argument('--bias-drones', default='', help='... on these drone ids, e.g. 2,3')
    ap.add_argument('--tick-jitter', type=float, default=0.004, help='timer jitter, wall s')
    ap.add_argument('--stall-prob', type=float, default=0.003, help='chance of a 60 ms CPU stall per tick')
    ap.add_argument('--takeoff-at', type=float, default=8.0, help='wall s after start')
    ap.add_argument('--radius', default='auto', help="spawn ring radius or 'auto'")
    # --- controller parameters (names = ROS params / launch args) ----------------
    for k, v in sph_core.COMMON_DEFAULTS.items():
        if k in ('robot_prefix', 'total_robots', 'run_id', 'run_label', 'log_dir', 'git_rev'):
            continue
        ap.add_argument('--' + k, type=type(v), default=None, help=f'(default {v})')
    a = ap.parse_args()

    cfg = dict(sph_core.COMMON_DEFAULTS)
    # same defaults the real launch uses through swarm.conf
    cfg.update({'particle_mass': 7000.0, 'gamma': 2.0, 'zetta_extra': 10.0, 'f_k': 0.0})
    for k in cfg:
        if hasattr(a, k) and getattr(a, k) is not None:
            cfg[k] = getattr(a, k)
    cfg['total_robots'] = a.n
    cfg['run_id'] = f"{time.strftime('%Y%m%d_%H%M%S')}_{a.label}"
    cfg['run_label'] = a.label
    cfg['git_rev'] = 'offline-twin'
    cfg['log_dir'] = a.out
    os.makedirs(a.out, exist_ok=True)

    import gen_config
    if a.radius == 'auto':
        radius = round(gen_config.auto_radius(a.n, cfg['particle_mass'], cfg['smoothing_h'],
                                              cfg['rho_0'], 0.3), 3)
    else:
        radius = float(a.radius)
    pos = gen_config.positions(a.n, radius)
    cfg['R'] = radius          # potential radius = spawn radius, as in swarm.conf (POT_R=auto)
    cfg['xc'] = cfg['yc'] = 0.0   # ring centred on the origin (like the spawn ring); f_k=0 by default

    rng = random.Random(a.seed)
    bias_ids = {int(s) for s in a.bias_drones.split(',') if s.strip()}
    drones = {}
    for i, (x, y) in enumerate(pos):
        r = math.hypot(x, y) or 1.0
        b = (a.bias * x / r, a.bias * y / r) if i in bias_ids else (0.0, 0.0)
        drones[f'cf_{i}'] = SimDrone(f'cf_{i}', x, y, a.tau, a.delay, b, cfg['fixed_z'], rng)
    names = list(drones)

    # --- simulated clock --------------------------------------------------------
    t_epoch0 = time.time()
    wall = [0.0]
    epoch_fn = lambda: t_epoch0 + wall[0]              # noqa: E731  (injected into the core)

    cores = []
    if a.mode == 'swarm':
        core, _ = sph_core.build_core(cfg, lambda n: list(n), density_on_rx=False, tag='swarm',
                                      epoch_fn=epoch_fn)
        cores.append(core)
    else:
        for i, nm in enumerate(names):
            c, _ = sph_core.build_core(cfg, lambda n, nm=nm: [nm], density_on_rx=True,
                                       tag=f'plain_{nm}', write_meta=(i == 0), epoch_fn=epoch_fn)
            cores.append(c)

    rtf_path = os.path.join(a.out, 'runs', cfg['run_id'], 'rtf.csv')
    rtf_f = open(rtf_path, 'w')
    rtf_f.write('epoch,sim_time,real_time,rtf\n')

    # --- wall-clock loop ----------------------------------------------------------
    DT = 0.001
    sim_t, rtf = 0.0, a.rtf
    period = cfg['control_period']
    next_tick, k_tick, last_exec = period, 1, 0.0
    exec_at = next_tick
    next_sim_tick, last_sim_tick = period, 0.0
    pending = []                                       # heap of (deliver_wall, seq, name, values)
    seq = 0
    next_rtf_log = 0.0
    takeoff_done = False
    steps = int(a.seconds / DT)
    for _ in range(steps):
        wall[0] += DT
        w = wall[0]
        if a.rtf_sigma > 0:
            rtf = min(1.5, max(0.05, rtf + rng.gauss(0, a.rtf_sigma * math.sqrt(DT)) +
                               0.2 * (a.rtf - rtf) * DT))
        dts = DT * rtf
        sim_t += dts
        if not takeoff_done and w >= a.takeoff_at:
            takeoff_done = True
            for d in drones.values():
                d.flying = True
        for d in drones.values():
            d.advance(sim_t, dts)
            if sim_t >= d.next_pub_sim:
                d.next_pub_sim += 1.0 / a.state_hz
                seq += 1
                heapq.heappush(pending, (w + a.rx_latency * (0.5 + rng.random()), seq,
                                         d.name, d.state(0.002, 0.01)))
        while pending and pending[0][0] <= w:
            _, _, nm, vals = heapq.heappop(pending)
            for c in cores:
                c.on_state(nm, w, vals)
        sim_mode = cfg['sim_clock'] == 'gz'
        if sim_mode:
            # controller ticks every `period` of SIMULATED time (what sim_clock=gz does for real)
            fire = sim_t >= next_sim_tick
            sk = (sim_t, sim_t - last_sim_tick) if fire else None
        else:
            fire = w >= exec_at
            sk = None
        if fire:
            if sim_mode:
                next_sim_tick += period
                last_sim_tick = sim_t
            for c in cores:
                out = c.step(w) if sk is None else c.step(w, sk[0], sk[1])
                for nm, cmd in out.items():
                    if cmd is not None:
                        drones[nm].command(sim_t, cmd[0], cmd[1])
            last_exec = w
            if not sim_mode:
                k_tick += 1
                next_tick = k_tick * period
                jitter = abs(rng.gauss(0, a.tick_jitter))
                if rng.random() < a.stall_prob:
                    jitter += 0.06
                exec_at = max(next_tick + jitter, last_exec + 0.001)
        if w >= next_rtf_log:
            next_rtf_log += 0.5
            rtf_f.write(f'{epoch_fn():.3f},{sim_t:.4f},{w:.4f},{rtf:.4f}\n')
    rtf_f.close()
    for c in cores:
        c.close(wall[0])
    run_dir = os.path.join(a.out, 'runs', cfg['run_id'])
    print(f'twin done: {a.n} drones, {a.seconds:.0f} wall-s = {sim_t:.1f} sim-s '
          f'(mean rtf {sim_t / a.seconds:.2f}), ring radius {radius} m')
    print(f'run folder: {run_dir}')
    print(f'analyze   : python3 {os.path.join(HERE, "analyze_run.py")} {run_dir}')


if __name__ == '__main__':
    main()
