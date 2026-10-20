"""The SPH control loop without ROS.

plain_sph.py (one node per drone) and swarm_sph.py (one node for all drones) are thin
wrappers: they feed state messages into SphCore.on_state(), call SphCore.step() from
a timer and publish what step() returns. Keeping the loop here means

  * both nodes behave identically and log identically,
  * src/offline_twin.py can run the very same loop against a simulated drone, so the
    logs and the analyzer can be checked without Gazebo.

Numerics are the original ones: density/pressure -> Particle.next_vel() ->
Particle.update_particle() -> speed clip. What was ADDED are switches whose defaults
reproduce the old behaviour exactly:

    stale_policy  what to do on a tick with no new state message from the drone
                    predict  (old) integrate again from the previous prediction
                    hold     re-send the previous command, no integration
                    skip     publish nothing
    start_gate    when the controller may act
                    none     (old) immediately
                    all_known  once every drone has reported
                    flight   once every drone has been above 80 % of fixed_z for 1 s
    sph_gain      scales the SPH force (0 = SPH off)  [set on the particles]
    dt_model      integration step of the SPH model (default = control period)
    profile       known test commands instead of SPH (CommandProfile)
    sim_clock     wall (old) | gz: tick every `period` of SIMULATED time (see sim_clock.py);
                  step() is then called with sim_t / sim_dt, the gate hold time and the
                  test profiles run in simulated seconds, LATE_TICK is not evaluated.
"""

import math
import os
import time

from drone_controller.particle import Particle
from drone_controller.particle3d import Particle3D
from drone_controller.potential import Potential
from drone_controller.potential3d import Potential3D
from drone_controller.sph_diag import CommandProfile, RunLogger, clip_norm

STALE_POLICIES = ('predict', 'hold', 'skip')
START_GATES = ('none', 'all_known', 'flight')
FLIGHT_GATE_TIMEOUT_S = 40.0     # gate 'flight': start anyway this long after the first tick

LOG_DIR = '/root/dev_ws/src/logs'

# parameters both nodes understand (node-specific defaults are passed in `overrides`)
COMMON_DEFAULTS = {
    'robot_prefix': 'cf_',
    'total_robots': 2,
    'fixed_z': 0.5,
    'particle_mass': 7000.0,
    'smoothing_h': 3.0,
    'rho_0': 1000.0,
    'gamma': 7.0,
    'alpha': 1.0,
    'beta': 2.0,
    'zetta_extra': 30.0,
    'xc': 2.0,
    'yc': 2.0,
    'R': 0.5,
    'f_beta': 1.0,
    'f_k': 2.0,
    'max_velocity': 0.5,        # m/s, speed limit of the command (real drones: start lower)
    # --- diagnostics / test switches (defaults == old behaviour) ---
    'sph_gain': 1.0,            # 0 = SPH force off (damping + external force only)
    'control_period': 0.05,     # s, timer period
    'dt_model': 0.0,            # s, SPH integration step; 0 = same as control_period
    'stale_policy': 'predict',  # predict | hold | skip
    'start_gate': 'flight',       # none | all_known | flight
    'test_profile': 'none',     # none | zero | step | square | sine | chirp
    'test_amp': 0.15,           # m/s
    'test_period': 4.0,         # s
    'test_axis': 'x',
    'test_delay': 3.0,          # s after the gate opens
    'test_duration': 60.0,      # s (chirp)
    'run_id': '',               # folder name under logs/runs/ (launch file fills it in)
    'run_label': '',
    'git_rev': '',
    'log_dir': LOG_DIR,
    # --- time base ---
    'sim_clock': 'wall',        # wall | gz   (gz = tick on Gazebo simulated time; simulation only)
    'gz_stats_topic': '',       # '' = auto-detect the world stats topic
    'keepalive_s': 0.1,         # sim_clock=gz: re-send the last command after this much wall time
    'poll_period': 0.005,       # s of wall time between checks for the next simulated tick
    # --- safety (REAL drones; all off by default) ---
    'platform': 'sim',          # sim | real  (only labels the run; the analyzer reads it)
    'fence_xmin': float('nan'), 'fence_xmax': float('nan'),   # m, world frame; nan = off
    'fence_ymin': float('nan'), 'fence_ymax': float('nan'),
    'state_timeout_s': 0.0,     # s without a state message from ANY drone -> trip (0 = off)
    # --- 3D mode (dim = 3: sphere potential + 3D SPH; dim = 2 ignores everything below) ---
    'dim': 2.0,                 # 2 | 3   (float so a launch argument can carry it)
    'zc': 1.0,                  # m, sphere centre height (3D)
    'max_vz': 0.25,             # m/s, climb / descent limit of the command (3D)
    'z_min': 0.25,              # m, floor: soft wall + the command never goes below it (3D)
    'z_max': 2.5,               # m, ceiling (3D)
    'z_margin': 0.3,            # m, the soft wall starts this far from z_min / z_max
    'z_wall_k': 2.0,            # m/s^2, wall force at the limit
    'z_stretch': 1.0,           # >1: drones keep more VERTICAL room (downwash); kernel uses dz / z_stretch
    # --- which target shape pulls the drones (see potential.py / potential3d.py) ---
    # default | circle | sphere | quartic   default = circle in 2D, sphere in 3D.  quartic = the rounded star
    # (2D) / the star loop on a tilted plane (3D); R is then its size constant, f_beta is ignored and f_k / 2 scales the force (f_k = 2: original).
    'potential': 'default',
    'obstacles':'none',
    'obs_sigma': 10.0,  
    'f_kt': 0.0,
    'f_g1': 0.0,         # G1: extra convergence gain per unit |alpha1| (quartic), 0 = off         
    'test_side': 1.0,           # m, side of the square of test_profile=box (centred on xc, yc)
}

SPH_MSG_LEN = 10                # /<drone>/sph message: x y z vx vy vz rho P c m

class SphCore:
    def __init__(self, names, egos, particles, potential, logger, *,
                 f_beta=1.0, f_k=2.0, period=0.05, dt_model=None, max_velocity=0.5,
                 stale_policy='predict', start_gate='none', fixed_z=0.5,
                 flight_z_frac=0.8, flight_hold_s=1.0,
                 profile=None, legacy_dir=None,
                 hb_period=5.0, nb_fresh_ms=None, epoch_fn=None, sim_clock='wall',
                 fence=None, state_timeout_s=0.0, dim=2, max_vz=0.25, z_min=0.25, z_max=2.5,
                 z_margin=0.3, z_wall_k=2.0):
        if stale_policy not in STALE_POLICIES:
            raise ValueError(f'stale_policy must be one of {STALE_POLICIES}')
        if start_gate not in START_GATES:
            raise ValueError(f'start_gate must be one of {START_GATES}')
        self.names = list(names)
        self.egos = list(egos)
        self.p = particles 
        self.obs = []
        self.potential = potential
        self.log = logger
        self._epoch = epoch_fn or time.time     # unix time; the offline twin passes its own clock
        self.f_beta, self.f_k = f_beta, f_k
        self.period = float(period)
        self.dt_model = float(dt_model) if dt_model else self.period
        self.max_velocity = max_velocity
        self.sim_clock = sim_clock
        self.fence = fence                       # (xmin, xmax, ymin, ymax) with None = open side
        self.state_timeout_s = float(state_timeout_s or 0.0)
        self.dim = int(dim)
        self.max_vz, self.z_min, self.z_max = float(max_vz), float(z_min), float(z_max)
        self.z_margin, self.z_wall_k = max(float(z_margin), 1e-6), float(z_wall_k)
        self.tripped = None                      # reason string once the safety latch fired
        self.landing = False                     # True once a landing was requested (latched)
        self.stale_policy, self.start_gate = stale_policy, start_gate
        self.fixed_z = fixed_z
        self.gate_z = flight_z_frac * fixed_z
        self.flight_hold_s = flight_hold_s
        self.profile = profile or CommandProfile('none')
        self.hb_period = hb_period
        self.nb_fresh_ms = nb_fresh_ms if nb_fresh_ms else 3000.0 * self.period

        self.known = set()
        self.rx = {n: {'count': 0, 't_last': None, 'meas': (0.0,) * 6} for n in self.names}
        self.es = {n: {'tick': 0, 'rx_at_tick': 0, 'stale_run': 0, 'clip_run': 0,
                       'last_cmd': (0.0,) * int(dim), 'last_cmd_raw': (0.0,) * int(dim),
                       'rho_state': None, 'last_state_evt': -1e9,
                       'hb': self._new_hb()} for n in self.egos}
        self.stats = {'ticks': 0, 'stale': 0, 'clipped': 0, 'late': 0, 'slow': 0}
        self._last_step_now = None
        self._gate_open = start_gate == 'none'
        self._gate_open_t = None
        self._flight_since = None
        self._gate_t0 = None                 # first tick (time base of the flight-gate timeout)
        self._last_gate_msg = -1e9
        self._prof_t0 = None
        self._prof_done = False
        self._throttle_t = {}
        self._throttle_n = {}
        self._hb_next = hb_period
        self._missing_warned = False

        for n in self.egos:
            self.log.open_drone(n, legacy_dir=legacy_dir)

    def add_obstacle(self, x, y, m, z=0.0):
        """Register a static obstacle particle: same Particle(3D) as a drone, heavier, never moves."""
        q = next(iter(self.p.values()))
        oid = f'obs_{len(self.obs)}'
        if self.dim == 3:
            ob = Particle3D(id=oid, x=x, y=y, z=z, h=q.h, m=m, rho_0=q.rho_0, gamma=q.gamma, alpha=q.alpha,
                        beta=q.beta, z_stretch=q.z_stretch, obstacle=True)
        else:
            ob = Particle(id=oid, x=x, y=y, h=q.h, m=m, rho_0=q.rho_0, gamma=q.gamma,
                        alpha=q.alpha, beta=q.beta, obstacle=True)
        self.obs.append(ob)
        self.log.event('INFO', 'OBSTACLE', f'{oid} at x={x:.3f} y={y:.3f} z={z:.3f} m={m:.0f}', t=0.0)
        return ob
    # ------------------------------------------------------------------ description
    def describe(self):
        """Goes into meta.json."""
        any_p = next(iter(self.p.values()))
        return {
            'names': self.names, 'egos': self.egos,
            'period_s': self.period, 'dt_model_s': self.dt_model,
            'max_velocity': self.max_velocity, 'sim_clock': self.sim_clock,
            'fence': self.fence, 'state_timeout_s': self.state_timeout_s,
            'stale_policy': self.stale_policy, 'start_gate': self.start_gate,
            'fixed_z': self.fixed_z, 'gate_z': self.gate_z,
            'f_beta': self.f_beta, 'f_k': self.f_k,
            'dim': self.dim,
            'potential': {'shape': self.potential.shape,
                          'xc': self.potential.xc, 'yc': self.potential.yc, 'R': self.potential.R,
                          'zc': getattr(self.potential, 'zc', None),
                          'a': getattr(self.potential, 'a', None), 'b': getattr(self.potential, 'b', None),
                          'c': getattr(self.potential, 'c', None)},
            'z_limits': ({'z_min': self.z_min, 'z_max': self.z_max, 'z_margin': self.z_margin,
                          'z_wall_k': self.z_wall_k, 'max_vz': self.max_vz,
                          'z_stretch': any_p.z_stretch} if self.dim == 3 else None),
            'sph': {'mass': any_p.m, 'h': any_p.h, 'rho_0': any_p.rho_0, 'gamma': any_p.gamma,
                    'alpha': any_p.alpha, 'beta': any_p.beta,
                    'zetta_extra': any_p.zetta_extra, 'sph_gain': any_p.sph_gain},
            'test_profile': self.profile.describe(),
        }

    # ------------------------------------------------------------------ inputs
    def on_state(self, name, now, values, hdr_lat_ms=None):
        if len(values) < 6:
            return
        x, y, z, vx, vy, vz = (float(v) for v in values[:6])
        r = self.rx[name]
        dt_ms = None if r['t_last'] is None else (now - r['t_last']) * 1e3
        first = r['t_last'] is None
        r['count'] += 1
        r['t_last'] = now
        r['meas'] = (x, y, z, vx, vy, vz)

        p = self.p[name]
        p.x, p.y = x, y
        if self.dim == 3:
            p.z = z
            p.vel = [vx, vy, vz]
        else:
            p.vel = [vx, vy]
        self.known.add(name)
        if first:
            self.log.event('INFO', 'FIRST_RX',
                           f'first state of {name}: x={x:.3f} y={y:.3f} z={z:.3f} '
                           f'({len(self.known)}/{len(self.names)} drones known)', t=now, name=name)
        # if self.density_on_rx:
        #     known = [self.p[n] for n in self.names if n in self.known] + self.obs
        #     p.rho = p.next_rho(known)
        #     p.P = p.next_pressure()
        if name in self.es:
            self.log.rx(name, {'t': now, 'epoch': self._epoch(), 'drone': name, 'rx_dt_ms': dt_ms,
                               'x': x, 'y': y, 'z': z, 'vx': vx, 'vy': vy, 'vz': vz,
                               'hdr_lat_ms': hdr_lat_ms})

    
    def on_sph(self, name, now, values):
        """SPH state published by another drone's node on /<name>/sph: x y z vx vy vz rho P c m.
        Position and velocity are handled as a state message; rho, P, c and m are the neighbour's
        OWN values (computed by its node), so they are not recomputed here (Pimenta et al., 2013)."""
        if name in self.egos or name not in self.p or len(values) < SPH_MSG_LEN:
            return
        self.on_state(name, now, values[:6])
        q = self.p[name]
        q.rho, q.P, q.c, q.m = (float(v) for v in values[6:SPH_MSG_LEN])

    def sph_state(self, name):
        """What this node publishes on /<name>/sph: measured x y z vx vy vz + its own rho P c m
        (None until the drone's own state has arrived)."""
        r = self.rx[name]
        if r['t_last'] is None:
            return None
        p = self.p[name]
        return [float(v) for v in r['meas']] + [float(p.rho), float(p.P), float(p.c), float(p.m)]

    def _support(self, ego, cands):
        """Particles of cands inside the kernel support of ego (distance < 2h of ego; in 3D the
        kernel metric, dz / z_stretch). Only these enter ego's density and acceleration."""
        r_sup = 2.0 * ego.h
        if self.dim == 3:
            return [q for q in cands if q.id != ego.id and ego.kdist(q) < r_sup]
        return [q for q in cands if q.id != ego.id and ego.distance_to(q) < r_sup]
    
    # ------------------------------------------------------------------ gate
    def _update_gate(self, now, tt=None):
        if self._gate_open:
            return True
        if self._gate_t0 is None:
            self._gate_t0 = tt
        missing = [n for n in self.names if n not in self.known]
        low = []
        if self.start_gate == 'all_known':
            ok = not missing
        else:   # flight
            low = [n for n in self.names if n in self.known and self.rx[n]['meas'][2] < self.gate_z]
            if not missing and not low:
                if self._flight_since is None:
                    self._flight_since = tt
                ok = (tt - self._flight_since) >= self.flight_hold_s
            else:
                self._flight_since = None
                ok = False
        if not ok and self.start_gate == 'flight' and tt - self._gate_t0 >= FLIGHT_GATE_TIMEOUT_S:
            ok = True
            self.log.event('WARN', 'GATE_TIMEOUT',
                           f'{FLIGHT_GATE_TIMEOUT_S:.0f} s without every drone at the flight altitude '
                           f'(below z={self.gate_z:.2f} m: {",".join(low) or "-"}; '
                           f'no state: {",".join(missing) or "-"}): starting anyway', t=now)
        if ok:
            self._gate_open, self._gate_open_t = True, now
            self.log.event('INFO', 'GATE_OPEN',
                           f'start gate "{self.start_gate}" open: controller now acts', t=now)
        elif now - self._last_gate_msg >= 5.0:
            self._last_gate_msg = now
            why = []
            if missing:
                why.append('no state yet from ' + ','.join(missing))
            if low:
                why.append(f'below z={self.gate_z:.2f} m: ' + ','.join(low))
            self.log.event('INFO', 'GATE_WAIT', '; '.join(why) or 'holding', t=now)
        return self._gate_open

    # ------------------------------------------------------------------ helpers
    def _throttled(self, key, now, every):
        """True if `key` may be reported now; also tells how many were suppressed."""
        last = self._throttle_t.get(key, -1e9)
        if now - last < every:
            self._throttle_n[key] = self._throttle_n.get(key, 0) + 1
            return False
        return True

    def _suppressed(self, key):
        n = self._throttle_n.pop(key, 0)
        return f' (+{n} similar suppressed)' if n else ''

    def _warn(self, code, msg, now, name, every=2.0):
        key = (code, name)
        if self._throttled(key, now, every):
            self.log.event('WARN', code, msg + self._suppressed(key), t=now, name=name)
            self._throttle_t[key] = now

    @staticmethod
    def _new_hb():
        return {'n': 0, 'rho': 0.0, 'P': 0.0, 'v2': 0.0, 'r': 0.0, 'stale': 0, 'clip': 0,
                'dt_max': 0.0, 'age_max': 0.0, 'rx': 0}

    # ------------------------------------------------------------------ one control tick
    def step(self, now, sim_t=None, sim_dt=None):
        """Run one control tick. Returns {drone: (vx, vy) | None} to publish
        (None = publish nothing this tick).
        now     wall-clock seconds (logging, ages)
        sim_t   simulated seconds (sim_clock=gz) or None; sim_dt = simulated time since the
                previous tick. The model always integrates dt_model (== the period by
                default), which equals sim_dt up to the ticker's jitter."""
        sim_driven = sim_t is not None
        tt = sim_t if sim_driven else now          # time base of gate hold + test profile
        t_perf = time.perf_counter()
        epoch = self._epoch()
        known = [self.p[n] for n in self.names if n in self.known]
        if not known:
            return {}
        wall_dt = None if self._last_step_now is None else (now - self._last_step_now) * 1e3
        self._last_step_now = now
        gate_open = self._update_gate(now, tt)

        field = known + self.obs                   # drones + obstacle particles
        for q in self.obs:                         # obstacles: from EVERY known drone (eq. 37)
            q.rho = q.next_rho(field)
            q.P = q.next_pressure()
        nbs = {}                                   # ego -> drones / obstacles within its 2h
        for name in self.egos:
            if name not in self.known:
                continue
            ego = self.p[name]
            nbs[name] = (self._support(ego, known), self._support(ego, self.obs))
            ego.rho = ego.next_rho([ego] + nbs[name][0] + nbs[name][1])   # own density only
            ego.P = ego.next_pressure()
            # for q in nbs[name][0]:                 # neighbours: recompute their rho / P / c here
            #     if q.id in self.egos:
            #         continue                       # swarm_sph: already computed as an ego
            #     q.rho = q.next_rho([q] + self._support(q, known) + self._support(q, self.obs))
            #     q.P = q.next_pressure()
                
        cx = sum(q.x for q in known) / len(known)
        cx = sum(q.x for q in known) / len(known)
        cy = sum(q.y for q in known) / len(known)
        cz = sum(q.z for q in known) / len(known) if self.dim == 3 else 0.0

        prof_s = None
        if self.profile.active and gate_open and len(self.known) == len(self.names):
            if self._prof_t0 is None:
                self._prof_t0 = tt
                self.log.event('INFO', 'TEST_START',
                               f'test profile starts: {self.profile.describe()}', t=now)
            prof_s = tt - self._prof_t0
            if prof_s > self.profile.total_time() and not self._prof_done:
                self._prof_done = True
                self.log.event('INFO', 'TEST_END',
                               'test profile finished (commanding 0 m/s now)', t=now)

        self._check_safety(now)

        # ---- phase 1: every ego's acceleration from the SAME snapshot ---------
        plan = []
        for name in self.egos:
            if name not in self.known:
                continue
            ego = self.p[name]
            st, rx = self.es[name], self.rx[name]
            rx_since = rx['count'] - st['rx_at_tick']
            st['rx_at_tick'] = rx['count']
            if self.dim == 3:
                ego.ext_force = self._ext3(ego)
                used = (ego.x, ego.y, ego.vel[0], ego.vel[1], ego.z, ego.vel[2])
            else:
                ego.ext_force = self.potential.force(ego.x, ego.y, self.f_beta, self.f_k)
                used = (ego.x, ego.y, ego.vel[0], ego.vel[1])
            neighbors, obs_nb = nbs[name]
            dvdt = ego.next_vel(neighbors + obs_nb)
            dists = [ego.distance_to(q) for q in neighbors]
            ages = [(now - self.rx[q.id]['t_last']) * 1e3 for q in neighbors]
            plan.append({
                'name': name, 'ego': ego, 'st': st, 'rx_since': rx_since, 'used': used,
                'dvdt': dvdt, 'terms': ego.last_terms, 'rho': ego.rho, 'P': ego.P, 'c': ego.c,
                'n_nb': len(neighbors),
                'nn': min(dists) if dists else float('nan'),
                'mean_nb': sum(dists) / len(dists) if dists else float('nan'),
                'n_nb_fresh': sum(1 for a in ages if a <= self.nb_fresh_ms),
                'max_age': max(ages) if ages else 0.0,
            })

        # ---- phase 2: decide the command, integrate, log -----------------------
        out = {}
        rows = []
        for e in plan:
            name, ego, st = e['name'], e['ego'], e['st']
            fresh = e['rx_since'] > 0
            if fresh:
                if st['stale_run'] >= 3:
                    self.log.event('WARN', 'STALE_RUN',
                                   f'{st["stale_run"]} consecutive ticks ({st["stale_run"] * self.period:.2f} s) '
                                   f'without a state message from {name}', t=now, name=name)
                st['stale_run'] = 0
            else:
                st['stale_run'] += 1
                self.stats['stale'] += 1

            if self.landing:
                src, raw = 'land', None                    # latched: publish nothing, the firmware lands
            elif self.tripped:
                src, raw = 'fence', (0.0,) * self.dim      # latched: hover command until restart
            elif not gate_open:
                src, raw = 'gate', None                    # publish nothing: let the takeoff happen
            
            elif prof_s is not None:
                meas = self.rx[name]['meas']
                src, raw = f'test:{self.profile.kind}', self.profile.value(
                    prof_s, pos=(meas[0], meas[1]), index=self.names.index(name))
                if self._prof_done:
                    raw = (0.0, 0.0)
                if self.dim == 3:
                    raw = tuple(raw) + (0.0,)
            elif self.profile.active:
                src, raw = 'gate', (0.0,) * self.dim    # waiting for the other drones
            elif fresh or self.stale_policy == 'predict':
                ego.update_particle(e['rho'], e['P'], e['dvdt'], self.dt_model)
                src = 'sph' if fresh else 'sph_pred'
                raw = tuple(ego.vel[:self.dim])
            elif self.stale_policy == 'hold':
                src, raw = 'hold', st['last_cmd_raw']
            else:
                src, raw = 'skip', None

            prev = st['last_cmd']
            if raw is None:
                cmd, clipped = None, False
                out[name] = None
            else:
                if self.dim == 3:
                    cmd = self._clip3(raw, self.rx[name]['meas'][2])
                    clipped = cmd != tuple(raw)
                else:
                    cmd = clip_norm(raw[0], raw[1], self.max_velocity)
                    clipped = (cmd[0] != raw[0]) or (cmd[1] != raw[1])
                out[name] = cmd
                st['last_cmd_raw'], st['last_cmd'] = raw, cmd

            self._detect(now, e, st, fresh, clipped, None if sim_driven else wall_dt, src)

            m = self.rx[name]['meas']
            u = e['used']
            t_ = e['terms']
            row = {
                't': now, 'epoch': epoch, 'tick': st['tick'], 'drone': name,
                'wall_dt_ms': wall_dt, 'sim_t': sim_t,
                'sim_dt_ms': None if sim_dt is None else sim_dt * 1e3,
                'dt_used_ms': self.dt_model * 1e3, 'gate': int(gate_open), 'src': src,
                'rx_since': e['rx_since'],
                'state_age_ms': (now - self.rx[name]['t_last']) * 1e3,
                'stale_run': st['stale_run'], 'n_known': len(self.known),
                'n_nb_fresh': e['n_nb_fresh'], 'max_nb_age_ms': e['max_age'],
                'meas_x': m[0], 'meas_y': m[1], 'meas_z': m[2],
                'meas_vx': m[3], 'meas_vy': m[4], 'meas_vz': m[5],
                'used_x': u[0], 'used_y': u[1], 'used_vx': u[2], 'used_vy': u[3],
                'pred_err': math.hypot(u[0] - m[0], u[1] - m[1]),
                'rho': e['rho'], 'P': e['P'], 'c': e['c'], 'n_nb': e['n_nb'],
                'nn_dist': e['nn'], 'mean_nb_dist': e['mean_nb'],
                'ap_x': t_['press'][0], 'ap_y': t_['press'][1],
                'av_x': t_['visc'][0], 'av_y': t_['visc'][1],
                'ad_x': t_['damp'][0], 'ad_y': t_['damp'][1],
                'ae_x': t_['ext'][0], 'ae_y': t_['ext'][1],
                'a_x': t_['total'][0], 'a_y': t_['total'][1],
                'cmd_raw_x': None if raw is None else raw[0],
                'cmd_raw_y': None if raw is None else raw[1],
                'cmd_x': None if cmd is None else cmd[0],
                'cmd_y': None if cmd is None else cmd[1],
                'clipped': int(clipped), 'prev_cmd_x': prev[0], 'prev_cmd_y': prev[1],
                'cen_x': cx, 'cen_y': cy,
                'r_cen': math.hypot(u[0] - cx, u[1] - cy),
                'r_pot': math.hypot(u[0] - self.potential.xc, u[1] - self.potential.yc),
            }
            if self.dim == 3:
                row.update({
                    'used_z': u[4], 'used_vz': u[5],
                    'ap_z': t_['press'][2], 'av_z': t_['visc'][2], 'ad_z': t_['damp'][2],
                    'ae_z': t_['ext'][2], 'a_z': t_['total'][2],
                    'cmd_raw_z': None if raw is None else raw[2], 'cmd_z': None if cmd is None else cmd[2],
                    'prev_cmd_z': prev[2], 'cen_z': cz,
                    'r_cen': math.sqrt((u[0] - cx) ** 2 + (u[1] - cy) ** 2 + (u[4] - cz) ** 2),
                    'r_pot': math.sqrt((u[0] - self.potential.xc) ** 2 + (u[1] - self.potential.yc) ** 2
                                       + (u[4] - self.potential.zc) ** 2),
                    'pred_err': math.sqrt((u[0] - m[0]) ** 2 + (u[1] - m[1]) ** 2 + (u[4] - m[2]) ** 2),
                })
            legacy = [now, e['rho'], e['P'], t_['ext'][0], t_['ext'][1],
                      ego.vel[0], ego.vel[1], u[0], u[1]]
            rows.append((name, row, legacy))
            st['tick'] += 1
            self.stats['ticks'] += 1
            if clipped:
                self.stats['clipped'] += 1
            self._heartbeat_add(st, e, row, fresh, clipped, wall_dt)

        step_ms = (time.perf_counter() - t_perf) * 1e3
        for name, row, legacy in rows:
            row['step_ms'] = step_ms
            self.log.tick(name, row, legacy=legacy)
        if step_ms > 0.5 * self.period * 1e3:
            self.stats['slow'] += 1
            self._warn('SLOW_STEP', f'control step took {step_ms:.1f} ms '
                       f'(period {self.period * 1e3:.0f} ms, {len(known)} drones)', now, None)
        self._heartbeat(now)
        return out

    # ------------------------------------------------------------------ 3D helpers
    def _ext3(self, ego):
        """Sphere-potential force + the soft floor / ceiling wall (z only)."""
        f = [float(v) for v in self.potential.force(ego.x, ego.y, ego.z, self.f_beta, self.f_k)]
        lo, hi, mg = self.z_min + self.z_margin, self.z_max - self.z_margin, self.z_margin
        if ego.z < lo:
            f[2] += self.z_wall_k * min((lo - ego.z) / mg, 1.0)
        elif ego.z > hi:
            f[2] -= self.z_wall_k * min((ego.z - hi) / mg, 1.0)
        return f

    def _clip3(self, raw, z_meas):
        """Horizontal speed limit, climb-rate limit, and a hard stop at the floor / ceiling."""
        vx, vy = clip_norm(raw[0], raw[1], self.max_velocity)
        vz = max(-self.max_vz, min(self.max_vz, raw[2]))
        if z_meas <= self.z_min and vz < 0.0:
            vz = 0.0
        elif z_meas >= self.z_max and vz > 0.0:
            vz = 0.0
        return (vx, vy, vz)

    # ------------------------------------------------------------------ safety latch
    def _check_safety(self, now):
        """Latch a stop (0 m/s to every drone, forever) when a drone leaves the fence or the
        state stream dies. The drones then hold position with the firmware's own loop;
        a human lands them. Nothing here can LAND or kill a drone."""
        if self.tripped:
            return
        why = None
        if self.fence is not None:
            xmin, xmax, ymin, ymax = self.fence
            for n in self.names:
                if n not in self.known:
                    continue
                x, y = self.rx[n]['meas'][0], self.rx[n]['meas'][1]
                if ((xmin is not None and x < xmin) or (xmax is not None and x > xmax) or
                        (ymin is not None and y < ymin) or (ymax is not None and y > ymax)):
                    why = f'{n} left the fence at x={x:.2f} y={y:.2f} (fence x[{xmin},{xmax}] y[{ymin},{ymax}])'
                    break
        if why is None and self.state_timeout_s > 0:
            for n in self.names:
                t_last = self.rx[n]['t_last']
                if t_last is not None and now - t_last > self.state_timeout_s:
                    why = f'no state from {n} for {now - t_last:.1f} s (limit {self.state_timeout_s} s)'
                    break
        if why:
            self._latch(why, now)

    def _latch(self, why, now):
        self.tripped = why
        self.log.event('ERROR', 'SAFETY_TRIP',
                       why + ' -> commanding 0 m/s to every drone from now on (latched). '
                       'Land the drones by hand.', t=now)

    def request_hover(self, now, why='manual hover requested (service)'):
        """Same latch as the fence, triggered by hand (ROS service ~/hover)."""
        if not self.tripped:
            self._latch(why, now)
    
    def request_land(self, now):
        """Stop commanding every ego, latched, so the firmware's high-level commander can land
        them (the node then calls notify_setpoints_stop + land). False if already landing."""
        if self.landing:
            return False
        self.landing = True
        self.log.event('INFO', 'LAND', 'land requested: the controller stops commanding '
                       '(latched); the drones land through the high-level commander', t=now)
        return True

    # ------------------------------------------------------------------ event detectors
    def _detect(self, now, e, st, fresh, clipped, wall_dt, src):
        name = e['name']
        if wall_dt is not None and wall_dt > 1.5 * self.period * 1e3:
            self.stats['late'] += 1
            self._warn('LATE_TICK', f'tick came {wall_dt:.0f} ms after the previous one '
                       f'(nominal {self.period * 1e3:.0f} ms)', now, name, every=5.0)
        if clipped:
            st['clip_run'] += 1
            if st['clip_run'] == 5:
                self.log.event('WARN', 'CLIP_START',
                               f'speed command saturated at {self.max_velocity} m/s for 5 ticks',
                               t=now, name=name)
        else:
            if st['clip_run'] >= 5:
                self.log.event('INFO', 'CLIP_END', f'saturation lasted {st["clip_run"]} ticks',
                               t=now, name=name)
            st['clip_run'] = 0
        rho0 = self.p[name].rho_0
        ratio = e['rho'] / rho0 if rho0 else 1.0
        regime = 'low' if ratio < 0.8 else ('high' if ratio > 1.2 else 'ok')
        if regime != st['rho_state']:
            if st['rho_state'] is not None and now - st['last_state_evt'] > 1.0:
                self.log.event('WARN' if regime != 'ok' else 'INFO', 'RHO_' + regime.upper(),
                               f'density {e["rho"]:.0f} = {ratio:.2f} x rho_0 '
                               f'(P={e["P"]:.0f}, n_nb={e["n_nb"]}, nearest {e["nn"]:.2f} m)',
                               t=now, name=name)
                st['last_state_evt'] = now
            st['rho_state'] = regime
        if src.startswith('sph') and not fresh:
            pe = math.hypot(e['used'][0] - self.rx[name]['meas'][0],
                            e['used'][1] - self.rx[name]['meas'][1])
            if pe > 0.05:
                self._warn('PRED_ERR', f'predicted position is {pe * 100:.0f} cm away from the '
                           f'last measured one', now, name, every=5.0)

    # ------------------------------------------------------------------ heartbeat
    def _heartbeat_add(self, st, e, row, fresh, clipped, wall_dt):
        h = st['hb']
        h['n'] += 1
        h['rho'] += e['rho']
        h['P'] += e['P']
        h['v2'] += row['meas_vx'] ** 2 + row['meas_vy'] ** 2
        h['r'] += row['r_pot']
        h['stale'] += 0 if fresh else 1
        h['clip'] += int(clipped)
        h['rx'] += e['rx_since']
        if wall_dt is not None:
            h['dt_max'] = max(h['dt_max'], wall_dt)
        h['age_max'] = max(h['age_max'], row['state_age_ms'])

    def _heartbeat(self, now):
        if now < self._hb_next:
            return
        win = self.hb_period
        self._hb_next = now + win
        for name in self.egos:
            h = self.es[name]['hb']
            if h['n'] == 0:
                continue
            n = h['n']
            self.log.event(
                'INFO', 'HB',
                f'rho={h["rho"] / n:7.1f} P={h["P"] / n:9.1f} |v|rms={math.sqrt(h["v2"] / n):.3f} '
                f'r_pot={h["r"] / n:.2f} stale={100.0 * h["stale"] / n:4.1f}% '
                f'rx={h["rx"] / win:4.1f}Hz clip={100.0 * h["clip"] / n:4.1f}% '
                f'dt_max={h["dt_max"]:.0f}ms age_max={h["age_max"]:.0f}ms',
                t=now, name=name)
            self.es[name]['hb'] = self._new_hb()

    # ------------------------------------------------------------------ shutdown
    def close(self, now=None):
        s = self.stats
        n = max(s['ticks'], 1)
        self.log.event('INFO', 'STOP',
                       f'ticks={s["ticks"]} stale={100.0 * s["stale"] / n:.1f}% '
                       f'clipped={100.0 * s["clipped"] / n:.1f}% late={s["late"]} slow={s["slow"]} '
                       f'warnings_so_far={self.log.counts.get("WARN", 0)}', t=now)
        self.log.close()


def _make_potential(cfg, dim):
    """The potential selected by cfg['potential']: default | circle | sphere | quartic."""
    kind = str(cfg.get('potential', 'default')).lower()
    if kind not in ('default', 'circle', 'sphere', 'quartic'):
        raise ValueError("potential must be default | circle | sphere | quartic")
    if (kind == 'circle' and dim != 2) or (kind == 'sphere' and dim != 3):
        raise ValueError(f"potential '{kind}' needs dim={2 if kind == 'circle' else 3}")
    if dim == 3:
        return Potential3D(xc=cfg['xc'], yc=cfg['yc'], zc=cfg['zc'], R=cfg['R'],
                       shape='quartic' if kind == 'quartic' else 'sphere', gain=cfg['f_k'] / 2.0,
                       kt=cfg.get('f_kt', 0.0), G_a1=cfg.get('f_g1', 0.0))
    return Potential(xc=cfg['xc'], yc=cfg['yc'], R=cfg['R'], shape='quartic' if kind == 'quartic' else 'circle',
                gain=cfg['f_k'] / 2.0, kt=cfg.get('f_kt', 0.0), G_a1=cfg.get('f_g1', 0.0))


def build_core(cfg, egos_from_names, tag, echo=None, write_meta=True, epoch_fn=None):   
    """Create particles, logger and SphCore from a parameter dict (ROS-free).
    echo(level, text) mirrors WARN/ERROR events to the ROS log (None offline).
    egos_from_names(names) -> list of the drones THIS node controls."""
    names = [f"{cfg['robot_prefix']}{i}" for i in range(int(cfg['total_robots']))]
    egos = egos_from_names(names)
    dim = int(round(float(cfg.get('dim', 2))))
    if dim not in (2, 3):
        raise ValueError('dim must be 2 or 3')
    if dim == 3:
        particles = {
            n: Particle3D(id=n, x=0.0, y=0.0, z=0.0, h=cfg['smoothing_h'], m=cfg['particle_mass'],
                          rho_0=cfg['rho_0'], gamma=cfg['gamma'], alpha=cfg['alpha'],
                          beta=cfg['beta'], zetta_extra=cfg['zetta_extra'],
                          sph_gain=cfg['sph_gain'], z_stretch=cfg['z_stretch'])
            for n in names
        }
        potential = _make_potential(cfg, 3)
    else:
        particles = {
            n: Particle(id=n, x=0.0, y=0.0, h=cfg['smoothing_h'], m=cfg['particle_mass'],
                        rho_0=cfg['rho_0'], gamma=cfg['gamma'], alpha=cfg['alpha'],
                        beta=cfg['beta'], zetta_extra=cfg['zetta_extra'],
                        sph_gain=cfg['sph_gain'])
            for n in names
        }
        potential = _make_potential(cfg, 2)

    run_id = cfg['run_id'] or time.strftime('%Y%m%d_%H%M%S')
    logger = RunLogger(cfg['log_dir'], run_id=run_id, tag=tag, echo=echo)
    profile = CommandProfile(cfg['test_profile'], amp=cfg['test_amp'], period=cfg['test_period'],
                             axis=cfg['test_axis'], delay=cfg['test_delay'],
                             duration=cfg['test_duration'],
                             side=cfg['test_side'], centre=(cfg['xc'], cfg['yc']))
    fv = [cfg['fence_xmin'], cfg['fence_xmax'], cfg['fence_ymin'], cfg['fence_ymax']] 
    fv = [None if (v is None or v != v) else float(v) for v in fv]       # nan -> open side
    fence = tuple(fv) if any(v is not None for v in fv) else None
    core = SphCore(
        names, egos, particles, potential, logger,
        f_beta=cfg['f_beta'], f_k=cfg['f_k'], period=cfg['control_period'],
        dt_model=cfg['dt_model'] or None, max_velocity=cfg['max_velocity'],
        stale_policy=cfg['stale_policy'], start_gate=cfg['start_gate'],
        fixed_z=cfg['fixed_z'], profile=profile,
        fence=fence, state_timeout_s=cfg['state_timeout_s'],
        legacy_dir=cfg['log_dir'], epoch_fn=epoch_fn, sim_clock=cfg['sim_clock'],
        dim=dim, max_vz=cfg['max_vz'], z_min=cfg['z_min'], z_max=cfg['z_max'],
        z_margin=cfg['z_margin'], z_wall_k=cfg['z_wall_k'],
    )
          # obstacles: 2D 'x,y' | 3D 'x,y,z' (one particle) or 'x,y' (a pillar: column from z_min to z_max, h apart)
    h = cfg['smoothing_h']
    w0 = 10.0 / (7.0 * math.pi * h ** 2) if dim == 2 else 1.0 / (math.pi * h ** 3)   # W(0, h), 2D / 3D
    m_obs = cfg['obs_sigma'] * cfg['rho_0'] / w0
    
    for item in str(cfg.get('obstacles', 'none')).replace(' ', '').split('/'):
        if not item or item.lower() == 'none':
            continue
        v = [float(s) for s in item.split(',')]
        if dim == 2 or len(v) == 3:
            core.add_obstacle(v[0], v[1], m_obs, z=v[2] if len(v) == 3 else 0.0)
        else:
            n = max(1, int(round((cfg['z_max'] - cfg['z_min']) / cfg['smoothing_h']))) + 1
            for k in range(n):
                core.add_obstacle(v[0], v[1], m_obs, z=cfg['z_min'] + k * (cfg['z_max'] - cfg['z_min']) / max(n - 1, 1))
    if write_meta:
        meta = {
            'run_id': run_id, 'label': cfg['run_label'], 'git_rev': cfg['git_rev'],
            'node': tag, 'started_epoch': (epoch_fn or time.time)(),
            'started': time.strftime('%Y-%m-%d %H:%M:%S'),
            'n_drones': len(names), 'params': {k: v for k, v in cfg.items()},
            'core': core.describe(),
            'pid': os.getpid(),
        }
        logger.write_meta(meta)
    logger.event('INFO', 'START',
                 f'{tag}: {len(names)} drones, egos={egos}, label="{cfg["run_label"]}", '
                 f'sph_gain={cfg["sph_gain"]} zeta={cfg["zetta_extra"]} f_k={cfg["f_k"]} '
                 f'gamma={cfg["gamma"]} m={cfg["particle_mass"]} policy={cfg["stale_policy"]} '
                 f'platform={cfg["platform"]} fence={fence} state_timeout={cfg["state_timeout_s"]} '
                 f'gate={cfg["start_gate"]} clock={cfg["sim_clock"]} vmax={cfg["max_velocity"]} dim={dim} '
                 f'test={profile.describe()}', t=0.0)
    return core, names