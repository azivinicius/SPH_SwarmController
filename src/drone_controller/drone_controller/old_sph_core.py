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
"""

import math
import os
import time

from drone_controller.particle import Particle
from drone_controller.potential import Potential
from drone_controller.sph_diag import CommandProfile, RunLogger, clip_norm

STALE_POLICIES = ('predict', 'hold', 'skip')
START_GATES = ('none', 'all_known', 'flight')

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
    # --- diagnostics / test switches (defaults == old behaviour) ---
    'sph_gain': 1.0,            # 0 = SPH force off (damping + external force only)
    'control_period': 0.05,     # s, timer period
    'dt_model': 0.0,            # s, SPH integration step; 0 = same as control_period
    'stale_policy': 'predict',  # predict | hold | skip
    'start_gate': 'none',       # none | all_known | flight
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
}


class SphCore:
    def __init__(self, names, egos, particles, potential, logger, *,
                 f_beta=1.0, f_k=2.0, period=0.05, dt_model=None, max_velocity=0.5,
                 stale_policy='predict', start_gate='none', fixed_z=0.5,
                 flight_z_frac=0.8, flight_hold_s=1.0,
                 density_on_rx=False, profile=None, legacy_dir=None,
                 hb_period=5.0, nb_fresh_ms=None, epoch_fn=None):
        if stale_policy not in STALE_POLICIES:
            raise ValueError(f'stale_policy must be one of {STALE_POLICIES}')
        if start_gate not in START_GATES:
            raise ValueError(f'start_gate must be one of {START_GATES}')
        self.names = list(names)
        self.egos = list(egos)
        self.p = particles                       # name -> Particle
        self.potential = potential
        self.log = logger
        self._epoch = epoch_fn or time.time     # unix time; the offline twin passes its own clock
        self.f_beta, self.f_k = f_beta, f_k
        self.period = float(period)
        self.dt_model = float(dt_model) if dt_model else self.period
        self.max_velocity = max_velocity
        self.stale_policy, self.start_gate = stale_policy, start_gate
        self.fixed_z = fixed_z
        self.gate_z = flight_z_frac * fixed_z
        self.flight_hold_s = flight_hold_s
        self.density_on_rx = density_on_rx
        self.profile = profile or CommandProfile('none')
        self.hb_period = hb_period
        self.nb_fresh_ms = nb_fresh_ms if nb_fresh_ms else 3000.0 * self.period

        self.known = set()
        self.rx = {n: {'count': 0, 't_last': None, 'meas': (0.0,) * 6} for n in self.names}
        self.es = {n: {'tick': 0, 'rx_at_tick': 0, 'stale_run': 0, 'clip_run': 0,
                       'last_cmd': (0.0, 0.0), 'last_cmd_raw': (0.0, 0.0),
                       'rho_state': None, 'last_state_evt': -1e9,
                       'hb': self._new_hb()} for n in self.egos}
        self.stats = {'ticks': 0, 'stale': 0, 'clipped': 0, 'late': 0, 'slow': 0}
        self._last_step_now = None
        self._gate_open = start_gate == 'none'
        self._gate_open_t = None
        self._flight_since = None
        self._last_gate_msg = -1e9
        self._prof_t0 = None
        self._prof_done = False
        self._throttle_t = {}
        self._throttle_n = {}
        self._hb_next = hb_period
        self._missing_warned = False

        for n in self.egos:
            self.log.open_drone(n, legacy_dir=legacy_dir)

    # ------------------------------------------------------------------ description
    def describe(self):
        """Goes into meta.json."""
        any_p = next(iter(self.p.values()))
        return {
            'names': self.names, 'egos': self.egos,
            'period_s': self.period, 'dt_model_s': self.dt_model,
            'max_velocity': self.max_velocity,
            'stale_policy': self.stale_policy, 'start_gate': self.start_gate,
            'fixed_z': self.fixed_z, 'gate_z': self.gate_z,
            'density_on_rx': self.density_on_rx,
            'f_beta': self.f_beta, 'f_k': self.f_k,
            'potential': {'xc': self.potential.xc, 'yc': self.potential.yc, 'R': self.potential.R},
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
        p.vel = [vx, vy]
        self.known.add(name)
        if first:
            self.log.event('INFO', 'FIRST_RX',
                           f'first state of {name}: x={x:.3f} y={y:.3f} z={z:.3f} '
                           f'({len(self.known)}/{len(self.names)} drones known)', t=now, name=name)
        if self.density_on_rx:
            known = [self.p[n] for n in self.names if n in self.known]
            p.rho = p.next_rho(known)
            p.P = p.next_pressure()
        if name in self.es:
            self.log.rx(name, {'t': now, 'epoch': self._epoch(), 'drone': name, 'rx_dt_ms': dt_ms,
                               'x': x, 'y': y, 'z': z, 'vx': vx, 'vy': vy, 'vz': vz,
                               'hdr_lat_ms': hdr_lat_ms})

    # ------------------------------------------------------------------ gate
    def _update_gate(self, now):
        if self._gate_open:
            return True
        missing = [n for n in self.names if n not in self.known]
        low = []
        if self.start_gate == 'all_known':
            ok = not missing
        else:   # flight
            low = [n for n in self.names if n in self.known and self.rx[n]['meas'][2] < self.gate_z]
            if not missing and not low:
                if self._flight_since is None:
                    self._flight_since = now
                ok = (now - self._flight_since) >= self.flight_hold_s
            else:
                self._flight_since = None
                ok = False
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
    def step(self, now):
        """Run one control tick. Returns {drone: (vx, vy) | None} to publish
        (None = publish nothing this tick)."""
        t_perf = time.perf_counter()
        epoch = self._epoch()
        known = [self.p[n] for n in self.names if n in self.known]
        if not known:
            return {}
        wall_dt = None if self._last_step_now is None else (now - self._last_step_now) * 1e3
        self._last_step_now = now
        gate_open = self._update_gate(now)

        if not self.density_on_rx:
            for q in known:
                q.rho = q.next_rho(known)
                q.P = q.next_pressure()

        cx = sum(q.x for q in known) / len(known)
        cy = sum(q.y for q in known) / len(known)

        prof_s = None
        if self.profile.active and gate_open and len(self.known) == len(self.names):
            if self._prof_t0 is None:
                self._prof_t0 = now
                self.log.event('INFO', 'TEST_START',
                               f'test profile starts: {self.profile.describe()}', t=now)
            prof_s = now - self._prof_t0
            if prof_s > self.profile.total_time() and not self._prof_done:
                self._prof_done = True
                self.log.event('INFO', 'TEST_END',
                               'test profile finished (commanding 0 m/s now)', t=now)

        # ---- phase 1: every ego's acceleration from the SAME snapshot ---------
        plan = []
        for name in self.egos:
            if name not in self.known:
                continue
            ego = self.p[name]
            st, rx = self.es[name], self.rx[name]
            rx_since = rx['count'] - st['rx_at_tick']
            st['rx_at_tick'] = rx['count']
            ego.ext_force = self.potential.force(ego.x, ego.y, self.f_beta, self.f_k)
            neighbors = [q for q in known if q.id != name]
            used = (ego.x, ego.y, ego.vel[0], ego.vel[1])
            dvdt = ego.next_vel(neighbors)
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

            if not gate_open:
                src, raw = 'gate', (0.0, 0.0)
            elif prof_s is not None:
                src, raw = f'test:{self.profile.kind}', self.profile.value(prof_s)
                if self._prof_done:
                    raw = (0.0, 0.0)
            elif self.profile.active:
                src, raw = 'gate', (0.0, 0.0)           # waiting for the other drones
            elif fresh or self.stale_policy == 'predict':
                ego.update_particle(e['rho'], e['P'], e['dvdt'], self.dt_model)
                src = 'sph' if fresh else 'sph_pred'
                raw = (ego.vel[0], ego.vel[1])
            elif self.stale_policy == 'hold':
                src, raw = 'hold', st['last_cmd_raw']
            else:
                src, raw = 'skip', None

            prev = st['last_cmd']
            if raw is None:
                cmd, clipped = None, False
                out[name] = None
            else:
                cmd = clip_norm(raw[0], raw[1], self.max_velocity)
                clipped = (cmd[0] != raw[0]) or (cmd[1] != raw[1])
                out[name] = cmd
                st['last_cmd_raw'], st['last_cmd'] = raw, cmd

            self._detect(now, e, st, fresh, clipped, wall_dt, src)

            m = self.rx[name]['meas']
            u = e['used']
            t_ = e['terms']
            row = {
                't': now, 'epoch': epoch, 'tick': st['tick'], 'drone': name,
                'wall_dt_ms': wall_dt, 'gate': int(gate_open), 'src': src,
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


def build_core(cfg, egos_from_names, density_on_rx, tag, echo=None, write_meta=True, epoch_fn=None):
    """Create particles, logger and SphCore from a parameter dict (ROS-free).
    echo(level, text) mirrors WARN/ERROR events to the ROS log (None offline).
    egos_from_names(names) -> list of the drones THIS node controls."""
    names = [f"{cfg['robot_prefix']}{i}" for i in range(int(cfg['total_robots']))]
    egos = egos_from_names(names)
    particles = {
        n: Particle(id=n, x=0.0, y=0.0, h=cfg['smoothing_h'], m=cfg['particle_mass'],
                    rho_0=cfg['rho_0'], gamma=cfg['gamma'], alpha=cfg['alpha'],
                    beta=cfg['beta'], zetta_extra=cfg['zetta_extra'],
                    sph_gain=cfg['sph_gain'])
        for n in names
    }
    potential = Potential(xc=cfg['xc'], yc=cfg['yc'], R=cfg['R'])

    run_id = cfg['run_id'] or time.strftime('%Y%m%d_%H%M%S')
    logger = RunLogger(cfg['log_dir'], run_id=run_id, tag=tag, echo=echo)
    profile = CommandProfile(cfg['test_profile'], amp=cfg['test_amp'], period=cfg['test_period'],
                             axis=cfg['test_axis'], delay=cfg['test_delay'],
                             duration=cfg['test_duration'])
    core = SphCore(
        names, egos, particles, potential, logger,
        f_beta=cfg['f_beta'], f_k=cfg['f_k'], period=cfg['control_period'],
        dt_model=cfg['dt_model'] or None, max_velocity=0.5,
        stale_policy=cfg['stale_policy'], start_gate=cfg['start_gate'],
        fixed_z=cfg['fixed_z'], density_on_rx=density_on_rx, profile=profile,
        legacy_dir=cfg['log_dir'], epoch_fn=epoch_fn,
    )
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
                 f'gate={cfg["start_gate"]} test={profile.describe()}', t=0.0)
    return core, names
