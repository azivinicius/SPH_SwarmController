#!/usr/bin/env python3
"""Checks analyze_run.py on synthetic runs whose answer is known:  python3 tests/test_analyzer.py"""
import json
import math
import os
import random
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, 'src', 'drone_controller'))
sys.path.insert(0, os.path.join(ROOT, 'src'))
from drone_controller.sph_diag import RunLogger          # noqa: E402
import analyze_run as ar                                  # noqa: E402

FAILS = []


def check(name, ok, detail=''):
    print(('PASS  ' if ok else 'FAIL  ') + name + (f'   {detail}' if detail else ''))
    if not ok:
        FAILS.append(name)


def make_run(kind, seconds=60.0, n=2, rtf=0.4, seed=1):
    """kind: calm | growing | plant (step response of a known first-order+delay drone)."""
    rng = random.Random(seed)
    d = tempfile.mkdtemp()
    lg = RunLogger(d, 'syn', tag='syn')
    lg.write_meta({'label': kind, 'n_drones': n, 'git_rev': 'syn', 'node': 'syn', 'params': {'f_k': 0.0},
                   'core': {'period_s': 0.05, 'dt_model_s': 0.05, 'fixed_z': 0.5, 'max_velocity': 0.5,
                            'stale_policy': 'predict', 'start_gate': 'flight',
                            'sph': {'mass': 7000.0, 'h': 3.0, 'rho_0': 1000.0, 'gamma': 2.0, 'zetta_extra': 10.0}}})
    for i in range(n):
        lg.open_drone(f'cf_{i}')
    G, DELAY, TAU = 0.9, 0.2, 0.3
    hist = {i: [] for i in range(n)}          # (t, cmd) history for the plant
    v = [0.0] * n
    t_epoch = 1.7e9
    next_rx = [0.0] * n
    last_rx = [None] * n
    rx_count = [0] * n
    used = [0] * n
    for k in range(int(seconds / 0.05)):
        t = k * 0.05
        for i in range(n):
            # command
            if kind == 'plant':
                c = 0.2 if 10 < t < 30 else 0.0
                c = c if i == 0 else -c
            else:
                c = 0.0
            hist[i].append((t, c))
            # plant state at this time (analytic update at 1 ms)
            if kind == 'plant':
                cd = 0.0
                for tt, cc in reversed(hist[i]):
                    if tt <= t - DELAY:
                        cd = cc
                        break
                for _ in range(50):
                    v[i] += (G * cd - v[i]) * (0.001 / TAU)
            elif kind == 'growing':
                v[i] = 0.003 * math.exp(0.08 * t) * math.sin(2 * math.pi * 1.2 * t + i)
            else:
                v[i] = rng.gauss(0, 0.003)
            # state message every 1/(20*rtf) wall seconds
            got = 0
            while t >= next_rx[i]:
                vm = v[i] + rng.gauss(0, 0.002)
                dt_ms = None if last_rx[i] is None else (next_rx[i] - last_rx[i]) * 1000
                lg.rx(f'cf_{i}', {'t': next_rx[i], 'epoch': t_epoch + next_rx[i], 'drone': f'cf_{i}',
                                  'rx_dt_ms': dt_ms, 'x': 0.0, 'y': 0.0, 'z': 0.5, 'vx': vm, 'vy': 0.0, 'vz': 0.0})
                last_rx[i] = next_rx[i]
                next_rx[i] += 1.0 / (20 * rtf)
                got += 1
            st = 0 if got else (used[i] + 1)
            used[i] = st
            lg.tick(f'cf_{i}', {'t': t, 'epoch': t_epoch + t, 'tick': k, 'drone': f'cf_{i}',
                                'wall_dt_ms': 50.0, 'step_ms': 0.2, 'gate': 1, 'src': 'test:step' if kind == 'plant' else 'sph',
                                'rx_since': got, 'state_age_ms': 20, 'stale_run': st, 'n_known': n,
                                'meas_x': 0.5 * i, 'meas_y': 0.0, 'meas_z': 0.5, 'meas_vx': v[i], 'meas_vy': 0.0,
                                'meas_vz': 0.0, 'used_x': 0.5 * i, 'used_y': 0.0, 'pred_err': 0.0,
                                'rho': 1000.0, 'P': 0.0, 'cmd_raw_x': c, 'cmd_raw_y': 0.0, 'cmd_x': c, 'cmd_y': 0.0,
                                'clipped': 0, 'ap_x': 0, 'ap_y': 0, 'av_x': 0, 'av_y': 0, 'ad_x': 0, 'ad_y': 0,
                                'ae_x': 0, 'ae_y': 0, 'a_x': 0, 'a_y': 0, 'cen_x': 0.25, 'cen_y': 0.0,
                                'r_cen': 0.25, 'r_pot': 0.25, 'used_vx': v[i], 'used_vy': 0.0})
    lg.close()
    # rtf.csv aligned in epoch
    with open(os.path.join(lg.run_dir, 'rtf.csv'), 'w') as f:
        f.write('epoch,sim_time,real_time,rtf\n')
        for s in range(int(seconds)):
            f.write(f'{t_epoch + s},{s * rtf},{s},{rtf}\n')
    return lg.run_dir


def analyse(path):
    A = ar.Analysis(ar.Run(path))
    A.run_all()
    return A


a = analyse(make_run('growing'))
check('growing oscillation is flagged GROWING', a.m.get('osc_growing') == 1, f'slope {a.m.get("rms_slope_per_s", float("nan")):.3f}')
check('oscillation frequency recovered (1.2 Hz wall = 3.0 Hz sim at RTF 0.4)',
      abs(a.m['osc_hz_wall'] - 1.2) < 0.15 and abs(a.m['osc_hz_sim'] - 3.0) < 0.4,
      f'{a.m["osc_hz_wall"]:.2f} wall / {a.m["osc_hz_sim"]:.2f} sim')
check('RTF read from rtf.csv', abs(a.m['rtf_mean'] - 0.4) < 1e-6)
check('stale fraction ~60 % at RTF 0.4', abs(a.m['stale_pct'] - 60.0) < 3.0, f'{a.m["stale_pct"]:.1f}')
check('diagnosis mentions TIME-BASE and GROWING-OSCILLATION',
      {'TIME-BASE', 'GROWING-OSCILLATION'} <= {c for _, c, _ in a.diag})

a = analyse(make_run('calm', seed=4))
check('calm run is not flagged as oscillating', a.m.get('osc_sustained') == 0 and a.m.get('osc_growing') == 0)

a = analyse(make_run('plant'))
g, dly, tau = a.m.get('track_gain'), a.m.get('track_delay_wall_s'), a.m.get('track_tau_wall_s')
check('step response: gain recovered (true 0.9)', g is not None and abs(g - 0.9) < 0.08, f'{g}')
check('step response: delay recovered (true 0.2 s)', dly is not None and abs(dly - 0.2) < 0.1, f'{dly}')
check('step response: time constant recovered (true 0.3 s, grid 0.25/0.4)', tau is not None and 0.15 <= tau <= 0.45, f'{tau}')
check('step response: R2 high', a.m.get('track_r2', 0) > 0.9)

# compare mode + figures run without error
d1, d2 = make_run('calm', seed=7), make_run('growing', seed=8)
out = tempfile.mkdtemp()
ar.compare([d1, d2], out)
check('--compare writes compare.md', os.path.exists(os.path.join(out, 'compare.md')))
A = analyse(d2)
ar.figures(A, os.path.join(out, 'figs'))
ar.write_outputs(A, os.path.join(out, 'figs'))
check('figures + report written', all(os.path.exists(os.path.join(out, 'figs', f)) for f in
                                      ('fig_overview.png', 'report.md', 'summary.txt', 'report.json')))
print('\nALL OK' if not FAILS else f'\n{len(FAILS)} FAILED: {FAILS}')
sys.exit(1 if FAILS else 0)
