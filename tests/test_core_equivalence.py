#!/usr/bin/env python3
"""Offline self-checks (no ROS, no Gazebo):  python3 tests/test_core_equivalence.py

1. SphCore with default switches reproduces the ORIGINAL control loops
   (swarm_sph: density per tick for all drones; plain_sph: density on message arrival)
   command-for-command, including ticks without a new state message.
2. The new switches do what they say (hold / skip / start gate / sph_gain / test profile).
3. RunLogger never raises when its folder cannot be written.
"""
import math
import os
import random
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'src', 'drone_controller'))

from drone_controller import sph_core                      # noqa: E402
from drone_controller.particle import Particle, grad_W, viscosity   # noqa: E402
from drone_controller.potential import Potential           # noqa: E402
from drone_controller.sph_diag import RunLogger, clip_norm  # noqa: E402

FAILS = []


def check(name, ok, detail=''):
    print(('PASS  ' if ok else 'FAIL  ') + name + (f'   {detail}' if detail else ''))
    if not ok:
        FAILS.append(name)


# ---- the original next_vel, verbatim (before the force split) ---------------------------
def old_next_vel(self, q_list):
    dvdt = [0.0, 0.0]
    for q in q_list:
        if q.id == self.id:
            continue
        dWdx, dWdy = grad_W(self, q)
        if self.rho < 1e-6 or q.rho < 1e-6:
            P_term = 0.0
        else:
            P_term = (self.P / self.rho ** 2) + (q.P / q.rho ** 2) + viscosity(self, q)
        dvdt[0] += q.m * P_term * dWdx
        dvdt[1] += q.m * P_term * dWdy
    dvdt[0] = -dvdt[0] - self.zetta_extra * self.vel[0] + self.ext_force[0]
    dvdt[1] = -dvdt[1] - self.zetta_extra * self.vel[1] + self.ext_force[1]
    return dvdt


def make_cfg(n, **kw):
    cfg = dict(sph_core.COMMON_DEFAULTS)
    cfg.update(total_robots=n, particle_mass=7000.0, gamma=2.0, zetta_extra=10.0, f_k=0.0,
               xc=0.0, yc=0.0, R=1.0, log_dir=tempfile.mkdtemp(), run_id='t', run_label='t')
    cfg.update(kw)
    return cfg


def parts(cfg, names):
    return {nm: Particle(nm, 0.0, 0.0, cfg['smoothing_h'], cfg['particle_mass'], cfg['rho_0'],
                         cfg['gamma'], cfg['alpha'], cfg['beta'], cfg['zetta_extra'])
            for nm in names}


def feed_sequence(n, ticks, seed):
    """per tick: {drone: [x,y,z,vx,vy,vz]} for the drones that report this tick."""
    rng = random.Random(seed)
    pos = [[1.5 * math.cos(2 * math.pi * i / n), 1.5 * math.sin(2 * math.pi * i / n)] for i in range(n)]
    seq = []
    for _ in range(ticks):
        msgs = {}
        for i in range(n):
            pos[i][0] += rng.gauss(0, 0.01)
            pos[i][1] += rng.gauss(0, 0.01)
            if rng.random() < 0.6:
                msgs[f'cf_{i}'] = [pos[i][0], pos[i][1], 0.5, rng.gauss(0, 0.05), rng.gauss(0, 0.05), 0.0]
        seq.append(msgs)
    return seq


# ---- 1a. swarm mode ------------------------------------------------------------------
def swarm_equivalence():
    n = 5
    cfg = make_cfg(n)
    names = [f'cf_{i}' for i in range(n)]
    ref = parts(cfg, names)
    known = set()
    core, _ = sph_core.build_core(cfg, lambda nm: list(nm), density_on_rx=False, tag='t')
    worst = 0.0
    for k, msgs in enumerate(feed_sequence(n, 400, 3)):
        for nm, v in msgs.items():
            core.on_state(nm, k * 0.05, v)
            p = ref[nm]
            p.x, p.y, p.vel = v[0], v[1], [v[3], v[4]]
            known.add(nm)
        out = core.step(k * 0.05 + 0.049)
        kn = [ref[nm] for nm in names if nm in known]
        if not kn:
            continue
        for p in kn:
            p.rho = p.next_rho(kn)
            p.P = p.next_pressure()
        plan = [(e, old_next_vel(e, [q for q in kn if q is not e])) for e in kn]
        for e, dv in plan:
            e.update_particle(e.rho, e.P, dv, 0.05)
            c = clip_norm(e.vel[0], e.vel[1], 0.5)
            worst = max(worst, abs(c[0] - out[e.id][0]), abs(c[1] - out[e.id][1]))
    core.close()
    check('swarm loop == original swarm_sph (400 ticks, 5 drones, 40 % stale)', worst < 1e-9, f'max diff {worst:.2e}')


# ---- 1b. plain mode (one node, ego cf_1, ext force on) ----------------------------------
def plain_equivalence():
    n = 4
    cfg = make_cfg(n, f_k=2.0, xc=0.3, yc=-0.2, R=1.4)
    names = [f'cf_{i}' for i in range(n)]
    ref = parts(cfg, names)
    pot = Potential(0.3, -0.2, 1.4)
    known = set()
    core, _ = sph_core.build_core(cfg, lambda nm: ['cf_1'], density_on_rx=True, tag='t')
    worst = 0.0
    for k, msgs in enumerate(feed_sequence(n, 400, 5)):
        for nm, v in msgs.items():
            core.on_state(nm, k * 0.05, v)
            p = ref[nm]
            p.x, p.y, p.vel = v[0], v[1], [v[3], v[4]]
            known.add(nm)
            kn = [ref[q] for q in names if q in known]
            p.rho = p.next_rho(kn)
            p.P = p.next_pressure()
        out = core.step(k * 0.05 + 0.049)
        if 'cf_1' not in known:
            continue
        kn = [ref[q] for q in names if q in known]
        ego = ref['cf_1']
        ego.ext_force = pot.force(ego.x, ego.y, 1.0, 2.0)
        dv = old_next_vel(ego, [q for q in kn if q.id != 'cf_1'])
        ego.update_particle(ego.rho, ego.P, dv, 0.05)
        c = clip_norm(ego.vel[0], ego.vel[1], 0.5)
        worst = max(worst, abs(c[0] - out['cf_1'][0]), abs(c[1] - out['cf_1'][1]))
    core.close()
    check('plain loop == original plain_sph (400 ticks, ext force on)', worst < 1e-9, f'max diff {worst:.2e}')


# ---- 2. switches --------------------------------------------------------------------------
def switches():
    n = 3
    base = [[1.0, 0.0, 0.5, 0.02, 0.0, 0.0], [-0.5, 0.87, 0.5, 0.0, 0.02, 0.0], [-0.5, -0.87, 0.5, 0.0, 0.0, 0.0]]

    def run(**kw):
        cfg = make_cfg(n, **kw)
        core, _ = sph_core.build_core(cfg, lambda nm: list(nm), density_on_rx=False, tag='t')
        for i, v in enumerate(base):
            core.on_state(f'cf_{i}', 0.0, v)
        return core

    c = run(stale_policy='hold')
    a = c.step(0.05)                                   # fresh
    b = c.step(0.10)                                   # stale
    check('stale_policy=hold re-sends the previous command', a['cf_0'] == b['cf_0'] and c.es['cf_0']['stale_run'] == 1)
    c = run(stale_policy='skip')
    c.step(0.05)
    check('stale_policy=skip publishes nothing', c.step(0.10)['cf_0'] is None)
    c = run(stale_policy='predict')
    c.step(0.05)
    check('stale_policy=predict keeps integrating', c.step(0.10)['cf_0'] is not None)

    c = run(start_gate='flight')
    out = c.step(0.05)
    check('start_gate=flight: zero command while on the ground', out['cf_0'] == (0.0, 0.0))
    for k in range(1, 40):                              # keep reporting z=0.5 for > 1 s
        for i, v in enumerate(base):
            c.on_state(f'cf_{i}', 0.05 * k, v)
        out = c.step(0.05 * k + 0.02)
    check('start_gate=flight opens after 1 s above 80 % of fixed_z', c._gate_open)

    c0 = run(sph_gain=0.0)
    out = c0.step(0.05)
    v = base[0][3:5]
    expect = (v[0] * (1 - 10 * 0.05), v[1] * (1 - 10 * 0.05))   # damping only: v + (-zeta v) dt
    check('sph_gain=0 -> pure damping', abs(out['cf_0'][0] - expect[0]) < 1e-9 and abs(out['cf_0'][1] - expect[1]) < 1e-9)

    c = run(test_profile='step', test_amp=0.2, test_period=2.0, test_delay=1.0, test_axis='y')
    seen = []
    for k in range(1, 120):
        t = 0.05 * k
        for i, vv in enumerate(base):
            c.on_state(f'cf_{i}', t, vv)
        seen.append((t, c.step(t + 0.02)['cf_2']))
    vals = {round(x[1][1], 3) for x in seen}
    check('test profile step: 0 -> +0.2 -> 0 on the chosen axis only', vals == {0.0, 0.2} and all(x[1][0] == 0.0 for x in seen))
    c.close()

    # ---- sim clock: gate hold and test profile run in SIMULATED seconds -------------
    c = run(start_gate='flight', sim_clock='gz')
    for k in range(1, 15):                              # 14 ticks = 0.7 sim s, but 140 wall s
        w, st_ = 10.0 * k, 0.05 * k
        for i, v in enumerate(base):
            c.on_state(f'cf_{i}', w, v)
        c.step(w, st_, 0.05)
    check('sim clock: gate hold counts simulated, not wall, seconds', not c._gate_open)
    for k in range(15, 30):
        w, st_ = 10.0 * k, 0.05 * k
        for i, v in enumerate(base):
            c.on_state(f'cf_{i}', w, v)
        c.step(w, st_, 0.05)
    check('sim clock: gate opens after 1 simulated second', c._gate_open)
    c.close()

    c = run(test_profile='step', test_amp=0.2, test_period=2.0, test_delay=1.0, sim_clock='gz')
    seen = []
    for k in range(1, 120):
        w, st_ = 7.0 * k, 0.05 * k                      # wall runs 140x faster than sim: irrelevant
        for i, vv in enumerate(base):
            c.on_state(f'cf_{i}', w, vv)
        seen.append((st_, c.step(w, st_, 0.05)['cf_0']))
    on = [t for t, v in seen if v[0] == 0.2]
    check('sim clock: test profile timed in simulated s (step 1.0 .. 3.0 s)',
          bool(on) and abs(on[0] - 1.05) < 0.11 and abs(on[-1] - 3.0) < 0.11, f'{on[0]:.2f}..{on[-1]:.2f}' if on else '')
    c.close()

    c = run(sim_clock='gz')
    for i, v in enumerate(base):
        c.on_state(f'cf_{i}', 0.0, v)
    c.step(5.0, 0.05, 0.05)
    c.step(40.0, 0.10, 0.05)                            # 35 wall s between ticks must NOT count as late
    check('sim clock: no LATE_TICK from wall gaps', c.stats['late'] == 0)
    c.close()

    # ---- safety latch (real drones) ---------------------------------------------------
    c = run(fence_xmin=-2.0, fence_xmax=2.0, fence_ymin=-2.0, fence_ymax=2.0)
    out = c.step(0.05)
    check('fence: nothing happens inside the box', c.tripped is None and out['cf_0'] != (0.0, 0.0))
    c.on_state('cf_1', 0.1, [2.6, 0.0, 0.5, 0.3, 0.0, 0.0])
    out = c.step(0.1)
    check('fence: drone outside the box -> latch, 0 m/s to ALL drones',
          c.tripped is not None and all(v == (0.0, 0.0) for v in out.values()), str(c.tripped))
    c.on_state('cf_1', 0.15, [0.0, 0.0, 0.5, 0.0, 0.0, 0.0])      # comes back: the latch stays
    out = c.step(0.15)
    check('fence: latch stays after the drone returns', all(v == (0.0, 0.0) for v in out.values()))
    c.close()

    c = run(state_timeout_s=0.5)
    c.step(0.05)
    c.on_state('cf_0', 0.5, base[0]); c.on_state('cf_1', 0.5, base[1])   # cf_2 went silent
    out = c.step(0.7)
    check('state timeout: one silent drone -> latch', c.tripped is not None and out['cf_0'] == (0.0, 0.0), str(c.tripped))
    c.close()

    c = run()
    c.step(0.05)
    c.request_hover(0.1)
    out = c.step(0.1)
    check('hover service: latch -> 0 m/s to all drones', all(v == (0.0, 0.0) for v in out.values()) and c.tripped)
    c.close()

    c = run()
    c.on_state('cf_1', 0.1, [50.0, 0.0, 0.5, 0.0, 0.0, 0.0])
    c.step(100.0)
    check('safety is off by default (old behaviour)', c.tripped is None)
    c.close()


# ---- 3. logger robustness -----------------------------------------------------------------
def logger_robust():
    msgs = []
    lg = RunLogger('/proc/definitely/not/writable', 'x', echo=lambda lvl, t: msgs.append(t))
    try:
        lg.open_drone('cf_0')
        lg.tick('cf_0', {'t': 0.0})
        lg.rx('cf_0', {'t': 0.0})
        lg.event('WARN', 'X', 'y')
        lg.close()
        ok = True
    except Exception as exc:                         # noqa: BLE001
        ok = False
        print(exc)
    check('RunLogger on an unwritable folder does not raise', ok)


if __name__ == '__main__':
    swarm_equivalence()
    plain_equivalence()
    switches()
    logger_robust()
    print('\nALL OK' if not FAILS else f'\n{len(FAILS)} FAILED: {FAILS}')
    sys.exit(1 if FAILS else 0)
