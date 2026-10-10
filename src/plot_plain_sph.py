#!/usr/bin/env python3
"""Plots of ONE run, shown in windows that stay open until you press Ctrl+C.

    python3 src/plot_plain_sph.py                      # newest run in src/logs/runs
    python3 src/plot_plain_sph.py src/logs/runs/<id>   # a given run (or `latest`)
    python3 src/plot_plain_sph.py <run> --no-open      # only write the PNGs, no windows, exit at once
    python3 src/plot_plain_sph.py <run> --out figs --collide 0.12 --close 0.30 --target-spacing 0.3

The PNGs go to <run>/plots/ . Each one opens in its own window (feh, else eog / display; install the
small one with `sudo apt install feh`, or set PLOT_VIEWER="cmd args"). The script keeps running while the
windows are open; Ctrl+C (or closing every window) ends it and closes all the windows it opened.

    00_summary.png      parameters (mass, h, rho0, gamma, zeta, gain, potential ...) and the key results
    01_trajectories.png map of the paths + target curve, altitude, and snapshots of the formation over time
                        (links between neighbours: green = target spacing +-20 %, orange tighter, blue looser)
    02_separation.png   closest approach vs time, collision / close-approach count, spacing histogram,
                        and the mass / spacing balance (ring density vs spacing for this mass, h, N)
    03_sph_dynamics.png rho/rho0, pressure P, external (potential) force, speed
    04_performance.png  RTF, busy cores per process group, CPU clock and temperature (needs rtf.csv / sys.csv)
    05_3d.png           3D runs only: paths + sphere / loop, altitude vs floor/ceiling, distance to the target

Distances are centre to centre in 3D (2D when there is no altitude). "Collision" = centre distance below
--collide (default 0.15 m: a Crazyflie is ~0.10 m across), "close approach" = below --close (default 0.25 m,
downwash range). One episode = one stretch below the threshold; gaps shorter than 0.5 s are merged.
Only the flight window is counted (all drones airborne, first 2 s skipped, up to a divergence if any).
"""
import argparse
import atexit
import math
import os
import shlex
import shutil
import signal
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import analyze_run as ar                      # noqa: E402  (also selects the Agg backend)
import matplotlib.pyplot as plt               # noqa: E402
from mpl_toolkits.mplot3d import Axes3D       # noqa: E402,F401  (registers the 3d projection)

GROUPS = ['gz', 'cf2', 'server', 'rviz', 'ctrl', 'tools', 'ros', 'other']
GROUP_LABEL = {'gz': 'Gazebo', 'cf2': 'firmware (cf2)', 'server': 'crazyflie_server', 'rviz': 'rviz2',
               'ctrl': 'SPH controller', 'tools': 'logging tools', 'ros': 'other ROS', 'other': 'other'}


# ----------------------------------------------------------------------------- helpers
def finite(a):
    return a is not None and len(a) > 0 and bool(np.isfinite(a).any())


def roll(x, k):
    x = np.asarray(x, dtype=float)
    if len(x) < k or k < 2:
        return x
    return np.convolve(np.nan_to_num(x), np.ones(k) / k, mode='valid')


def episodes(t, mask, merge_s=0.5):
    """[(t_start, t_end)] of contiguous True stretches; gaps shorter than merge_s are bridged."""
    out = []
    i, n = 0, len(mask)
    while i < n:
        if mask[i]:
            j = i
            while j + 1 < n and mask[j + 1]:
                j += 1
            if out and t[i] - out[-1][1] <= merge_s:
                out[-1] = (out[-1][0], t[j])
            else:
                out.append((t[i], t[j]))
            i = j + 1
        else:
            i += 1
    return out


def tfmt(v, nd=2, unit=''):
    return 'n/a' if v is None or (isinstance(v, float) and not math.isfinite(v)) else f'{v:.{nd}f}{unit}'


VIEWERS = [('feh', ['--scale-down', '--auto-zoom', '--title', '%f']), ('eog', ['--new-instance']),
           ('display', []), ('ristretto', []), ('gpicview', [])]


def find_viewer():
    """Command that opens ONE image in its own window and stays alive while the window is open.
    PLOT_VIEWER="cmd args" overrides the search."""
    env = os.environ.get('PLOT_VIEWER')
    if env:
        return shlex.split(env)
    for exe, extra in VIEWERS:
        if shutil.which(exe):
            return [exe] + extra
    return None


def show_and_wait(paths):
    """Open every image in its own window and keep running until Ctrl+C (or until every window was closed
    by hand); then close all the windows this script opened."""
    cmd = find_viewer()
    if cmd is None:
        print('(no image viewer that can be closed from here: install one with  sudo apt install feh ;'
              ' using xdg-open, those windows stay open when the script ends)')
        for pth in paths:
            subprocess.Popen(['xdg-open', pth], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return
    procs = []

    def close_all():
        for pr in procs:
            if pr.poll() is None:
                try:
                    os.killpg(pr.pid, signal.SIGTERM)
                except OSError:
                    pass
        t_end = time.time() + 1.5
        while time.time() < t_end and any(pr.poll() is None for pr in procs):
            time.sleep(0.05)
        for pr in procs:
            if pr.poll() is None:
                try:
                    os.killpg(pr.pid, signal.SIGKILL)
                except OSError:
                    pass

    atexit.register(close_all)
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: sys.exit(0))
    for pth in paths:
        try:
            procs.append(subprocess.Popen(cmd + [pth], start_new_session=True,
                                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        except OSError as exc:
            print(f'(could not open {pth}: {exc})')
    if not procs:
        return
    t0 = time.time()
    print(f'{len(procs)} windows open ({cmd[0]}). Ctrl+C here closes them all.')
    try:
        while True:
            time.sleep(0.5)
            if all(pr.poll() is not None for pr in procs):
                if time.time() - t0 < 3.0:
                    print('(the viewer handed the images to an already running window; Ctrl+C to finish)')
                    while True:
                        time.sleep(1.0)
                break
    except KeyboardInterrupt:
        pass
    print('closing windows')


# ----------------------------------------------------------------------------- data
class RunData:
    def __init__(self, path, collide, close, skip=2.0):
        self.run = ar.Run(path)
        self.R = self.run
        self.names = self.run.names
        self.meta = self.run.meta or {}
        core = self.meta.get('core', {})
        self.P = self.meta.get('params', {})
        self.sph = core.get('sph', {})
        self.core = core
        self.fixed_z = core.get('fixed_z', 0.5)
        self.max_v = core.get('max_velocity', 0.5)
        self.period = core.get('period_s', 0.05)
        self.n = len(self.names)
        self.collide, self.close = collide, close
        # ---- time base: seconds since the first tick (epoch), else the node clock -----------------
        eps = [self.R.ticks[n]['epoch'] for n in self.names if 'epoch' in self.R.ticks[n]]
        eps = [e for e in eps if finite(e)]
        self.E0 = float(min(np.nanmin(e) for e in eps)) if eps else None
        # ---- per-drone series on the shared time base --------------------------------------------
        self.S = {}
        for n in self.names:
            T = self.R.ticks[n]
            t = np.asarray(T['t'], dtype=float)
            if self.E0 is not None and 'epoch' in T and finite(T['epoch']):
                t = np.asarray(T['epoch'], dtype=float) - self.E0
            x, y, z = (T.get('meas_x'), T.get('meas_y'), T.get('meas_z'))
            if not finite(x):
                x, y = T.get('used_x'), T.get('used_y')
            self.S[n] = dict(t=t, x=np.asarray(x, dtype=float) if x is not None else None,
                             y=np.asarray(y, dtype=float) if y is not None else None,
                             z=np.asarray(z, dtype=float) if z is not None else None)
        # ---- flight window -------------------------------------------------------------------------
        self.t_end = max(float(np.nanmax(s['t'])) for s in self.S.values())
        self.t_beg = min(float(np.nanmin(s['t'])) for s in self.S.values())
        self.w0, self.w1 = self.t_beg, self.t_end
        self.flight_known = False
        try:
            A = ar.Analysis(self.run, skip=skip)
            shift = 0.0
            if self.E0 is not None:
                offs = [np.nanmedian(self.R.ticks[n]['epoch'] - self.R.ticks[n]['t']) for n in self.names
                        if 'epoch' in self.R.ticks[n] and finite(self.R.ticks[n]['epoch'])]
                shift = float(np.median(offs)) - self.E0 if offs else 0.0
            self.w0, self.w1 = A.w0 + shift, A.w1 + shift
            self.flight_known = A.flight_start is not None
            self.diverged = A.diverged_at is not None
            self.diverged_why = A.diverged_why
            self.A = A
        except Exception as exc:                                   # old logs, odd runs: use the whole run
            print(f'(flight window not available: {exc})')
            self.A, self.diverged, self.diverged_why = None, False, ''
        # ---- pair distances on a common grid ---------------------------------------------------------
        self.grid = None
        self.pairs = {}
        self.dmin = None
        self.nn = {}
        self._pair_distances()
        # ---- RTF and system monitor on the same time base --------------------------------------------
        self.rtf_t = self.rtf_v = None
        if self.run.rtf is not None and self.E0 is not None and finite(self.run.rtf.get('epoch')):
            self.rtf_t = np.asarray(self.run.rtf['epoch'], dtype=float) - self.E0
            self.rtf_v = np.asarray(self.run.rtf['rtf'], dtype=float)
        # sys.csv: read it here, so this script also works with an older analyze_run.py (no `sysm` attribute)
        self.sys = getattr(self.run, 'sysm', None)
        sp = os.path.join(self.run.dir, 'sys.csv')
        if self.sys is None and os.path.exists(sp) and os.path.getsize(sp) > 60:
            self.sys = ar.read_csv_cols(sp)
        self.sys_t = (np.asarray(self.sys['epoch'], dtype=float) - self.E0) if (self.sys and self.E0 is not None) else None

    def _pair_distances(self):
        ok = [n for n in self.names if finite(self.S[n]['x']) and finite(self.S[n]['y'])]
        if len(ok) < 2:
            return
        lo = max(float(np.nanmin(self.S[n]['t'])) for n in ok)
        hi = min(float(np.nanmax(self.S[n]['t'])) for n in ok)
        if hi - lo < 1.0:
            return
        g = np.arange(lo, hi, 0.1)
        pos = {}
        for n in ok:
            s = self.S[n]
            m = np.isfinite(s['x']) & np.isfinite(s['y']) & np.isfinite(s['t'])
            tt = s['t'][m]
            order = np.argsort(tt)
            cols = [np.interp(g, tt[order], s['x'][m][order]), np.interp(g, tt[order], s['y'][m][order])]
            if finite(s['z']):
                mz = m & np.isfinite(s['z'])
                tz = s['t'][mz]
                oz = np.argsort(tz)
                cols.append(np.interp(g, tz[oz], s['z'][mz][oz]))
            pos[n] = np.vstack(cols)
        dims = min(p.shape[0] for p in pos.values())
        self.dims3 = dims == 3
        for i, a in enumerate(ok):
            for b in ok[i + 1:]:
                d = pos[a][:dims] - pos[b][:dims]
                self.pairs[(a, b)] = np.sqrt((d ** 2).sum(axis=0))
        D = np.vstack(list(self.pairs.values()))
        self.grid, self.dmin = g, D.min(axis=0)
        for n in ok:
            ds = [v for (a, b), v in self.pairs.items() if n in (a, b)]
            self.nn[n] = np.vstack(ds).min(axis=0)

    # window mask on the grid
    def gmask(self):
        return (self.grid >= self.w0) & (self.grid <= self.w1)

    def equilibrium(self):
        m, h, rho0 = self.sph.get('mass'), self.sph.get('h'), self.sph.get('rho_0') or 1000.0
        if m is None or h is None or self.n < 2:
            return float('nan'), float('nan')
        r = ar.equilibrium_ring_radius(self.n, m, h, rho0)
        return r, (2 * r * math.sin(math.pi / self.n) if math.isfinite(r) else float('nan'))

    # ---- numbers used by the summary and printed on the console ----------------------------------
    def results(self):
        res = {}
        if self.grid is not None:
            w = self.gmask()
            if w.any():
                dm = self.dmin[w]
                tw = self.grid[w]
                res['dmin'] = float(dm.min())
                res['dmin_med'] = float(np.median(dm))
                res['ep_collide'] = episodes(tw, dm < self.collide)
                res['ep_close'] = episodes(tw, dm < self.close)
                res['t_below_collide'] = float((dm < self.collide).sum() * 0.1)
                res['t_below_close'] = float((dm < self.close).sum() * 0.1)
                nn = np.concatenate([v[w] for v in self.nn.values()])
                res['nn_med'] = float(np.median(nn))
                res['nn_p5'], res['nn_p95'] = float(np.percentile(nn, 5)), float(np.percentile(nn, 95))
        res['window'] = (self.w0, self.w1)
        if self.rtf_t is not None:
            m = (self.rtf_t >= self.w0) & (self.rtf_t <= self.w1) & np.isfinite(self.rtf_v)
            if m.any():
                res['rtf_mean'], res['rtf_min'] = float(np.mean(self.rtf_v[m])), float(np.min(self.rtf_v[m]))
        if self.sys_t is not None:
            m = (self.sys_t >= self.w0) & (self.sys_t <= self.w1)
            if m.sum() >= 3:
                s = self.sys
                res['busy'] = float(np.mean(np.asarray(s['cores_total'])[m]))
                res['ncpu'] = float(s['ncpu'][0])
                res['ncore'] = float(s['ncore'][0])
                res['clock'] = float(np.nanmean(np.asarray(s['freq_mhz'])[m]))
                res['temp'] = float(np.nanmax(np.asarray(s['temp_c'])[m]))
                for g_ in GROUPS:
                    res['cpu_' + g_] = float(np.mean(np.asarray(s['cpu_' + g_])[m]))
        # control loop
        steps, stale, clipped, wdt = [], [], [], []
        for n in self.names:
            T = self.R.ticks[n]
            t = self.S[n]['t']
            w = (t >= self.w0) & (t <= self.w1)
            if not w.any():
                continue
            if 'step_ms' in T:
                steps.append(np.asarray(T['step_ms'])[w])
            if 'rx_since' in T:
                stale.append((np.asarray(T['rx_since'])[w] == 0).astype(float))
            if 'clipped' in T:
                clipped.append(np.asarray(T['clipped'])[w])
            if 'wall_dt_ms' in T:
                wdt.append(np.asarray(T['wall_dt_ms'])[w])
        if steps:
            res['step_p95'] = float(np.nanpercentile(np.concatenate(steps), 95))
        if stale:
            res['stale_pct'] = float(100 * np.nanmean(np.concatenate(stale)))
        if clipped:
            res['clip_pct'] = float(100 * np.nanmean(np.concatenate(clipped)))
        if wdt:
            w_ = np.concatenate(wdt)
            res['late_pct'] = float(100 * np.nanmean(w_ > 1.5 * 1000 * self.period))
        return res


# ----------------------------------------------------------------------------- figures
def color_map(names):
    cyc = plt.rcParams['axes.prop_cycle'].by_key()['color']
    return {n: cyc[i % len(cyc)] for i, n in enumerate(names)}


def shade_window(ax, D):
    ax.axvspan(D.w0, D.w1, color='0.5', alpha=0.07, lw=0)
    ax.grid(True, alpha=0.3)


def quartic_info(D):
    """The quartic target (centre, R, a, b, c) when the run used it, else None. Old runs without a, b, c use the defaults."""
    pt = D.core.get('potential') or {}
    if str(pt.get('shape', D.P.get('potential', 'default'))).lower() != 'quartic' \
            and str(D.P.get('potential', 'default')).lower() != 'quartic':
        return None
    g = lambda k, d: float(pt.get(k) if pt.get(k) is not None else d)
    return dict(xc=g('xc', D.P.get('xc', 0.0)), yc=g('yc', D.P.get('yc', 0.0)), R=g('R', D.P.get('R', 1.0)),
                a=g('a', 4.0), b=g('b', 6.0), c=g('c', 4.0))


def quartic_curve(q, n=400):
    """x, y of the curve a*dx^4 - b*dx^2*dy^2 + c*dy^4 = R (one point per angle)."""
    t = np.linspace(0, 2 * math.pi, n)
    ct, st = np.cos(t), np.sin(t)
    r = (q['R'] / (q['a'] * ct ** 4 - q['b'] * ct ** 2 * st ** 2 + q['c'] * st ** 4)) ** 0.25
    return q['xc'] + r * ct, q['yc'] + r * st


def quartic_alpha1(q, x, y):
    dx, dy = x - q['xc'], y - q['yc']
    return q['a'] * dx ** 4 - q['b'] * dx ** 2 * dy ** 2 + q['c'] * dy ** 4 - q['R']



def fig_summary(D, res, outdir):
    fig = plt.figure(figsize=(13, 8.5))
    ax = fig.add_axes([0.02, 0.02, 0.96, 0.9])
    ax.axis('off')
    P, S = D.P, D.sph
    r_eq, s_eq = D.equilibrium()
    spawn = None
    n0 = D.names[0]
    for n in D.names:
        s = D.S[n]
        if finite(s['x']):
            i0 = int(np.nanargmin(s['t']))
            spawn = spawn or []
            spawn.append((s['x'][i0], s['y'][i0]))
    if spawn and len(spawn) > 1:
        c = np.mean(spawn, axis=0)
        spawn_r = float(np.mean([math.hypot(a - c[0], b - c[1]) for a, b in spawn]))
    else:
        spawn_r = float('nan')

    def line(k, v):
        return f'{k:<26}{v}'

    left = [
        'RUN', line('folder', D.run.name), line('label', str(D.meta.get('label', ''))),
        line('code', str(D.meta.get('git_rev', '?'))), line('mode', str(D.meta.get('node', '?'))),
        line('drones', D.n), line('platform / clock', f'{P.get("platform", "sim")} / {D.core.get("sim_clock", "wall")}'),
        '',
        'SPH PARAMETERS', line('mass m', S.get('mass', P.get('particle_mass', '?'))),
        line('smoothing h (m)', S.get('h', P.get('smoothing_h', '?'))),
        line('rho_0', S.get('rho_0', P.get('rho_0', '?'))), line('gamma', S.get('gamma', P.get('gamma', '?'))),
        line('alpha / beta', f'{P.get("alpha", "?")} / {P.get("beta", "?")}'),
        line('zetta_extra (damping)', S.get('zetta_extra', P.get('zetta_extra', '?'))),
        line('sph_gain', P.get('sph_gain', '?')),
        line('equilibrium ring radius', tfmt(r_eq, 3, ' m')),
        line('equilibrium spacing', tfmt(s_eq, 3, ' m')),
        line('spawn ring radius', tfmt(spawn_r, 3, ' m')),
        '',
        'CONTROL / POTENTIAL', line('period', f'{D.period * 1000:.0f} ms'), line('dt_model', f'{D.core.get("dt_model_s", "?")} s'),
        line('max_velocity', f'{D.max_v} m/s'), line('stale policy / gate', f'{D.core.get("stale_policy", "?")} / {D.core.get("start_gate", "?")}'),
        line('fixed_z', f'{D.fixed_z} m'), line('f_k / f_beta', f'{P.get("f_k", "?")} / {P.get("f_beta", "?")}'),
        line('potential xc, yc, R', f'{P.get("xc", "?")}, {P.get("yc", "?")}, {P.get("R", "?")}'),
        line('test profile', str(P.get('test_profile', 'none'))),
    ]
    right = ['RESULTS (flight window)']
    w0, w1 = res['window']
    right.append(line('window', f'{w0:.1f} .. {w1:.1f} s' + ('' if D.flight_known else '  (whole run: flight start unknown)')))
    if D.diverged:
        right.append(line('DIVERGED', D.diverged_why))
    right += ['', 'SEPARATION']
    if 'dmin' in res:
        right += [line('closest approach', f'{res["dmin"]:.3f} m'),
                  line('median closest', f'{res["dmin_med"]:.3f} m'),
                  line('nearest-neighbour', f'median {res["nn_med"]:.3f}  (p5 {res["nn_p5"]:.3f}, p95 {res["nn_p95"]:.3f}) m'),
                  line(f'collisions (< {D.collide:g} m)', f'{len(res["ep_collide"])} episode(s), {res["t_below_collide"]:.1f} s below'),
                  line(f'close (< {D.close:g} m)', f'{len(res["ep_close"])} episode(s), {res["t_below_close"]:.1f} s below')]
        if math.isfinite(s_eq):
            right.append(line('median NN / equilibrium', f'{res["nn_med"] / s_eq:.2f}'))
    else:
        right.append('no position data')
    right += ['', 'LOOP AND SIMULATOR',
              line('RTF mean / min', f'{tfmt(res.get("rtf_mean"))} / {tfmt(res.get("rtf_min"))}'),
              line('step cost p95', tfmt(res.get('step_p95'), 2, ' ms')),
              line('late ticks (>1.5 period)', tfmt(res.get('late_pct'), 1, ' %')),
              line('stale ticks', tfmt(res.get('stale_pct'), 1, ' %')),
              line('command saturated', tfmt(res.get('clip_pct'), 1, ' %'))]
    right += ['', 'MACHINE (sys.csv)']
    if 'busy' in res:
        right += [line('busy threads', f'{res["busy"]:.2f} of {res["ncpu"]:.0f}  ({res["ncore"]:.0f} physical cores)'),
                  line('clock / temp max', f'{res["clock"]:.0f} MHz / {res["temp"]:.0f} C')]
        top = sorted(((res['cpu_' + g], g) for g in GROUPS), reverse=True)[:4]
        right.append(line('biggest users (cores)', ', '.join(f'{GROUP_LABEL[g]} {v:.2f}' for v, g in top[:2])))
        right.append(line('', ', '.join(f'{GROUP_LABEL[g]} {v:.2f}' for v, g in top[2:4])))
    else:
        right.append('no sys.csv in this run')
    ax.text(0.0, 1.0, '\n'.join(left), va='top', ha='left', family='monospace', fontsize=9.5, transform=ax.transAxes)
    ax.text(0.5, 1.0, '\n'.join(right), va='top', ha='left', family='monospace', fontsize=9.5, transform=ax.transAxes)
    fig.suptitle(f'{D.run.name}   summary', fontsize=13, x=0.02, ha='left')
    p = os.path.join(outdir, '00_summary.png')
    fig.savefig(p, dpi=110)
    plt.close(fig)
    return p


def fig_trajectories(D, outdir, target=0.3, nsnap=4):
    """Top: map of the paths (+ target curve) and altitude.  Bottom: snapshots of the formation over the
    flight window; a line joins every pair closer than 1.6x the target spacing (green = within +-20 % of
    the target, orange = tighter, blue = looser)."""
    C = color_map(D.names)
    P = D.P
    fig = plt.figure(figsize=(14, 11))
    outer = fig.add_gridspec(2, 1, height_ratios=[1.5, 1], hspace=0.22)
    top = outer[0].subgridspec(1, 2, width_ratios=[1.25, 1], wspace=0.18)
    am = fig.add_subplot(top[0, 0])
    az = fig.add_subplot(top[0, 1])
    xs, ys = [], []
    pot_on = (bool(P.get('f_k')) and all(P.get(k) is not None for k in ('xc', 'yc', 'R'))
              and str(P.get('potential', 'default')).lower() != 'quartic')   # quartic: not a circle
    for n in D.names:
        s = D.S[n]
        if not finite(s['x']):
            continue
        t, x, y = s['t'], s['x'], s['y']
        w = (t >= D.w0) & (t <= D.w1)
        am.plot(x, y, '-', color=C[n], lw=0.8, alpha=0.3)
        if w.any():
            am.plot(x[w], y[w], '-', color=C[n], lw=1.8, label=n)
            am.plot(x[w][0], y[w][0], 'o', color=C[n], mec='k', ms=7)
            am.plot(x[w][-1], y[w][-1], 's', color=C[n], mec='k', ms=7)
            am.annotate(n, (x[w][-1], y[w][-1]), fontsize=7, xytext=(4, 4), textcoords='offset points')
        xs += list(x[np.isfinite(x)])
        ys += list(y[np.isfinite(y)])
    if not xs:
        plt.close(fig)
        return None
    if pot_on:
        xc, yc, R = float(P['xc']), float(P['yc']), float(P['R'])
        xs += [xc - R, xc + R]
        ys += [yc - R, yc + R]
    qi = quartic_info(D)
    if qi:
        qx, qy = quartic_curve(qi)
        xs += [float(qx.min()), float(qx.max())]
        ys += [float(qy.min()), float(qy.max())]
    span = max(max(xs) - min(xs), max(ys) - min(ys), 1.0)
    mg = 0.15 * span
    x0, x1, y0, y1 = min(xs) - mg, max(xs) + mg, min(ys) - mg, max(ys) + mg

    def target_curve(ax, lw):
        if pot_on:
            th = np.linspace(0, 2 * math.pi, 400)
            ax.plot(xc + R * np.cos(th), yc + R * np.sin(th), 'k--', lw=lw, label=f'target circle R={R:g}')
            ax.plot(xc, yc, 'k+', ms=10, mew=1.5)
        if qi:
            ax.plot(qx, qy, 'k--', lw=lw, label=f'quartic target R={qi["R"]:g}')
            ax.plot(qi['xc'], qi['yc'], 'k+', ms=10, mew=1.5)

    target_curve(am, 1.6)
    am.set_xlim(x0, x1)
    am.set_ylim(y0, y1)
    am.set_aspect('equal')
    am.set_xlabel('x (m)')
    am.set_ylabel('y (m)')
    am.set_title('paths: bold = flight window, faint = takeoff / rest   (o start, square end)')
    am.grid(True, alpha=0.3)
    am.legend(fontsize=7, loc='best', ncol=2)
    for n in D.names:
        s = D.S[n]
        if finite(s['z']):
            az.plot(s['t'], s['z'], color=C[n], lw=1)
    az.axhline(D.fixed_z, color='k', ls=':', lw=1)
    az.set_ylabel('altitude z (m)')
    az.set_xlabel('time (s)')
    az.set_title(f'altitude (target {D.fixed_z} m)')
    shade_window(az, D)

    # formation snapshots
    ok = [n for n in D.names if finite(D.S[n]['x']) and finite(D.S[n]['y'])]
    if ok and D.grid is not None:
        g = D.grid
        gm = (g >= D.w0) & (g <= D.w1)
        g = g[gm] if gm.sum() > nsnap else g
        X = np.vstack([np.interp(g, D.S[n]['t'], D.S[n]['x']) for n in ok])
        Y = np.vstack([np.interp(g, D.S[n]['t'], D.S[n]['y']) for n in ok])
        idx = np.unique(np.linspace(0, len(g) - 1, nsnap).astype(int))
        bot = outer[1].subgridspec(1, len(idx), wspace=0.12)
        for k, i in enumerate(idx):
            ax = fig.add_subplot(bot[0, k])
            target_curve(ax, 1.0)
            for a in range(len(ok)):
                for b in range(a + 1, len(ok)):
                    d = math.hypot(X[a, i] - X[b, i], Y[a, i] - Y[b, i])
                    if d < 1.6 * target:
                        col = 'tab:green' if abs(d - target) <= 0.2 * target else ('tab:orange' if d < target else 'tab:blue')
                        ax.plot([X[a, i], X[b, i]], [Y[a, i], Y[b, i]], color=col, lw=1.6, alpha=0.8, zorder=1)
            for a, n in enumerate(ok):
                ax.scatter(X[a, i], Y[a, i], s=60, color=C[n], edgecolor='k', zorder=3)
                ax.annotate(n.replace('cf_', ''), (X[a, i], Y[a, i]), xytext=(4, 4), textcoords='offset points', fontsize=8)
            ax.set_xlim(x0, x1)
            ax.set_ylim(y0, y1)
            ax.set_aspect('equal')
            ax.set_title(f't = {g[i]:.1f} s', fontsize=10)
            ax.grid(True, alpha=0.3)
            if k:
                ax.set_yticklabels([])
    fig.suptitle(f'{D.run.name}   trajectories and formation   (links: green = {target:g} m +-20 %, orange = tighter, blue = looser)',
                 x=0.02, y=0.94, ha='left', fontsize=13)
    p = os.path.join(outdir, '01_trajectories.png')
    fig.savefig(p, dpi=110, bbox_inches='tight')
    plt.close(fig)
    return p


def fig_separation(D, res, outdir):
    if D.grid is None:
        return None
    C = color_map(D.names)
    fig = plt.figure(figsize=(14, 9))
    gs = fig.add_gridspec(2, 2, height_ratios=[1.2, 1], hspace=0.3, wspace=0.22)
    a1 = fig.add_subplot(gs[0, :])
    a2 = fig.add_subplot(gs[1, 0])
    a3 = fig.add_subplot(gs[1, 1])
    r_eq, s_eq = D.equilibrium()
    # (a) closest approach vs time
    if D.n <= 6:
        for (a, b), d in D.pairs.items():
            a1.plot(D.grid, d, lw=0.7, alpha=0.45, label=f'{a}-{b}')
    a1.plot(D.grid, D.dmin, 'k', lw=1.6, label='closest pair')
    a1.axhline(D.collide, color='r', ls='-', lw=1.2, label=f'collision < {D.collide:g} m')
    a1.axhline(D.close, color='orange', ls='--', lw=1.2, label=f'close < {D.close:g} m')
    if math.isfinite(s_eq):
        a1.axhline(s_eq, color='g', ls=':', lw=1.4, label=f'equilibrium spacing {s_eq:.2f} m')
    if 'ep_collide' in res:
        for t0, t1 in res['ep_collide']:
            a1.axvspan(t0 - 0.05, t1 + 0.05, color='r', alpha=0.25, lw=0)
        for t0, t1 in res['ep_close']:
            a1.axvspan(t0 - 0.05, t1 + 0.05, color='orange', alpha=0.12, lw=0)
    shade_window(a1, D)
    a1.set_ylabel(f'centre distance, {"3D" if D.dims3 else "2D"} (m)')
    a1.set_xlabel('time (s)')
    ttl = f'closest approach: {tfmt(res.get("dmin"), 3, " m")}'
    if 'ep_collide' in res:
        ttl += f'   |   collisions: {len(res["ep_collide"])}   close approaches: {len(res["ep_close"])}'
    a1.set_title(ttl)
    a1.legend(fontsize=7, ncol=4, loc='upper right')
    a1.set_ylim(0, max(1.0, min(float(np.nanpercentile(D.dmin, 99)) * 1.4, 6.0)))
    # (b) nearest-neighbour histogram in the flight window
    w = D.gmask()
    if w.any():
        nn = np.concatenate([v[w] for v in D.nn.values()])
        hi = max(float(np.nanpercentile(nn, 99)) * 1.1, D.close * 1.5)
        a2.hist(nn, bins=np.linspace(0, hi, 40), color='0.4')
        a2.axvline(D.collide, color='r', lw=1.2)
        a2.axvline(D.close, color='orange', ls='--', lw=1.2)
        if math.isfinite(s_eq):
            a2.axvline(s_eq, color='g', ls=':', lw=1.4, label='equilibrium spacing')
        a2.axvline(float(np.median(nn)), color='b', lw=1.2, label=f'median {np.median(nn):.2f} m')
        a2.legend(fontsize=8)
    a2.set_xlabel('nearest-neighbour distance (m), all drones, flight window')
    a2.set_ylabel('samples')
    a2.set_title('spacing the swarm actually kept')
    a2.grid(True, alpha=0.3)
    # (c) mass / spacing balance: ring density vs spacing for this mass, h, N
    m, h, rho0 = D.sph.get('mass'), D.sph.get('h'), D.sph.get('rho_0') or 1000.0
    if m is not None and h is not None and D.n >= 2:
        sp = np.linspace(0.05, max(3.0 * h, 1.5 * (s_eq if math.isfinite(s_eq) else 1.0), 1.0), 300)
        dens = np.array([ar.ring_density(D.n, s / (2 * math.sin(math.pi / D.n)), m, h) for s in sp])
        a3.plot(sp, dens / rho0, 'k', lw=1.5, label=f'ring density, N={D.n}, m={m:g}, h={h:g}')
        a3.axhline(1.0, color='g', lw=1, ls='--', label='rho0 (pressure = 0)')
        if math.isfinite(s_eq):
            a3.axvline(s_eq, color='g', ls=':', lw=1.2)
        if 'nn_med' in res:
            a3.axvline(res['nn_med'], color='b', lw=1.2, label=f'measured median {res["nn_med"]:.2f} m')
        a3.axvspan(0, D.close, color='orange', alpha=0.12, label=f'closer than {D.close:g} m')
        a3.set_ylim(0, 4)
        a3.legend(fontsize=8)
    a3.set_xlabel('neighbour spacing on an evenly spaced ring (m)')
    a3.set_ylabel('rho / rho0')
    a3.set_title('mass / h / N balance: density > rho0 pushes apart, < rho0 pulls together')
    a3.grid(True, alpha=0.3)
    fig.suptitle(f'{D.run.name}   separation and collisions', x=0.02, ha='left', fontsize=13)
    p = os.path.join(outdir, '02_separation.png')
    fig.savefig(p, dpi=115, bbox_inches='tight')
    plt.close(fig)
    return p


def fig_sph(D, outdir):
    """rho / rho0, pressure P, external (potential) force, speed."""
    C = color_map(D.names)
    rho0 = D.sph.get('rho_0') or 1000.0
    fig, ax = plt.subplots(4, 1, figsize=(12, 12), sharex=True)

    def norm_of(T, k):
        comps = [np.asarray(T[f'{k}_{c}'], dtype=float) for c in 'xyz' if f'{k}_{c}' in T]
        comps = [c for c in comps if finite(c)]            # the z column is empty in 2D runs
        return np.sqrt(sum(c ** 2 for c in comps)) if comps else None

    for n in D.names:
        T = D.R.ticks[n]
        t = D.S[n]['t']
        if 'rho' in T:
            ax[0].plot(t, np.asarray(T['rho']) / rho0, color=C[n], lw=1, label=n)
        if 'P' in T and finite(T['P']):
            ax[1].plot(t, np.asarray(T['P'], dtype=float), color=C[n], lw=1)          # pressure P of the particle
        v = norm_of(T, 'ae')                                                          # external force |F| (m/s^2, per unit mass)
        if v is not None and len(v) == len(t):
            ax[2].plot(t, v, color=C[n], lw=1)
        if 'cmd_raw_x' in T:
            ax[3].plot(t, np.hypot(T['cmd_raw_x'], T['cmd_raw_y']), color=C[n], lw=0.8, alpha=0.8)
        if 'meas_vx' in T and finite(T['meas_vx']):
            ax[3].plot(t, np.hypot(T['meas_vx'], T['meas_vy']), color=C[n], lw=0.8, ls=':')
    ax[0].axhspan(0.9, 1.1, color='g', alpha=0.1)
    ax[0].axhline(1.0, color='g', lw=0.8, ls='--')
    ax[0].set_ylabel('rho / rho0  (green = +-10 %)')
    ax[0].legend(fontsize=7, ncol=min(D.n, 6))
    ax[1].set_ylabel('pressure P of each particle')
    ax[1].axhline(0, color='k', lw=0.8)
    ax[2].set_ylabel('external force |F| (potential), m/s^2')
    ax[3].axhline(D.max_v, color='r', ls=':', lw=1)
    from matplotlib.lines import Line2D
    ax[3].legend(handles=[Line2D([], [], color='k', lw=1, label='command (before clip)'),
                          Line2D([], [], color='k', lw=1, ls=':', label='measured'),
                          Line2D([], [], color='r', lw=1, ls=':', label='max_velocity')],
                 fontsize=8, ncol=3, loc='upper right')
    ax[3].set_ylabel('speed (m/s)')
    ax[3].set_xlabel('time (s)')
    for a in ax:
        shade_window(a, D)
    ax[0].set_title(f'{D.run.name}   SPH dynamics   (m={D.sph.get("mass", "?")}, h={D.sph.get("h", "?")}, '
                    f'gain={D.P.get("sph_gain", "?")}, zeta={D.sph.get("zetta_extra", "?")})', loc='left')
    fig.tight_layout()
    p = os.path.join(outdir, '03_sph_dynamics.png')
    fig.savefig(p, dpi=110)
    plt.close(fig)
    return p


def fig_performance(D, outdir):
    """RTF on its own panel, then CPU load and clock / temperature (one figure)."""
    have_rtf = D.rtf_t is not None
    have_sys = D.sys is not None and D.sys_t is not None
    if not (have_rtf or have_sys):
        return None
    rows = (1 if have_rtf else 0) + (2 if have_sys else 0)
    fig, ax = plt.subplots(rows, 1, figsize=(12, 3.6 * rows), sharex=True, squeeze=False)
    ax = list(ax[:, 0])
    i = 0
    if have_rtf:
        a = ax[i]
        i += 1
        a.plot(D.rtf_t, D.rtf_v, 'k', lw=1)
        a.axhline(1.0, color='g', lw=0.8, ls='--')
        a.axhline(0.9, color='orange', lw=0.8, ls=':', label='0.9 = reliable')
        a.axhline(0.6, color='r', lw=0.8, ls=':', label='0.6 = do not trust')
        a.set_ylim(0, max(1.15, float(np.nanmax(D.rtf_v)) * 1.1))
        a.set_ylabel('RTF (sim s per wall s)')
        a.legend(fontsize=8)
    if have_sys:
        s, t = D.sys, D.sys_t
        a = ax[i]
        i += 1
        ys = [np.asarray(s['cpu_' + g]) for g in GROUPS]
        labels = [GROUP_LABEL[g] for g in GROUPS]
        keep = [k for k, y in enumerate(ys) if float(np.nanmax(y)) > 0.02]
        a.stackplot(t, [ys[k] for k in keep], labels=[labels[k] for k in keep], alpha=0.85)
        a.plot(t, np.asarray(s['cores_total']), 'k', lw=1.2, label='whole machine')
        a.axhline(float(s['ncpu'][0]), color='r', ls='--', lw=1, label=f'{s["ncpu"][0]:.0f} threads')
        a.axhline(float(s['ncore'][0]), color='r', ls=':', lw=1, label=f'{s["ncore"][0]:.0f} physical cores')
        a.set_ylabel('busy cores, stacked by group')
        a.legend(fontsize=7, ncol=4, loc='upper left')
        a.set_ylim(0, float(s['ncpu'][0]) * 1.5)
        a = ax[i]
        a.plot(t, np.asarray(s['freq_mhz']), 'b', lw=1)
        a.set_ylabel('mean CPU clock (MHz)', color='b')
        a3 = a.twinx()
        a3.plot(t, np.asarray(s['temp_c']), 'r', lw=1)
        a3.set_ylabel('hottest zone (C)', color='r')
    ax[-1].set_xlabel('time (s)')
    for a in ax:
        shade_window(a, D)
    ax[0].set_title(f'{D.run.name}   simulator speed and machine load', loc='left')
    fig.tight_layout()
    p = os.path.join(outdir, '04_performance.png')
    fig.savefig(p, dpi=110)
    plt.close(fig)
    return p


def fig_3d(D, outdir):
    """3D runs only (dim = 3): final 3D snapshot with the potential sphere, altitude of every drone against
    the floor / ceiling and the sphere's z band, and the distance of each drone to the sphere surface."""
    if int(D.core.get('dim', 2) or 2) != 3:
        return None
    ok = [n for n in D.names if finite(D.S[n]['z']) and finite(D.S[n]['x'])]
    if not ok:
        return None
    P = D.P
    pot = D.core.get('potential', {})
    xc, yc, zc, R = pot.get('xc'), pot.get('yc'), pot.get('zc'), pot.get('R')
    if str(P.get('potential', 'default')).lower() == 'quartic':
        R = None                                  # the quartic loop is not a sphere: draw no sphere
    zl = D.core.get('z_limits') or {}
    C = color_map(ok)
    fig = plt.figure(figsize=(14, 8.5))
    gs = fig.add_gridspec(2, 2, width_ratios=[1, 1.15], hspace=0.3, wspace=0.18)
    a3 = fig.add_subplot(gs[:, 0], projection='3d')
    az = fig.add_subplot(gs[0, 1])
    ar_ = fig.add_subplot(gs[1, 1], sharex=az)
    if None not in (xc, yc, zc, R):
        u, v = np.meshgrid(np.linspace(0, 2 * math.pi, 40), np.linspace(0, math.pi, 20))
        a3.plot_wireframe(xc + R * np.cos(u) * np.sin(v), yc + R * np.sin(u) * np.sin(v), zc + R * np.cos(v),
                          color='0.7', lw=0.4, alpha=0.6)
    qi = quartic_info(D)
    if qi:
        qx, qy = quartic_curve(qi)
        a3.plot(qx, qy, (qx + qy) / 2.0, 'k--', lw=1.6)        # the loop lies on the plane z = (x + y) / 2
    for n in ok:
        s_ = D.S[n]
        a3.plot(s_['x'], s_['y'], s_['z'], color=C[n], lw=0.8, alpha=0.6)
        a3.scatter(s_['x'][-1], s_['y'][-1], s_['z'][-1], color=C[n], s=45, edgecolor='k')
        az.plot(s_['t'], s_['z'], color=C[n], lw=1.2, label=n)
        if qi:
            ar_.plot(s_['t'], np.abs(quartic_alpha1(qi, s_['x'], s_['y'])), color=C[n], lw=1.0)
        elif None not in (xc, yc, zc, R):
            d = np.sqrt((s_['x'] - xc) ** 2 + (s_['y'] - yc) ** 2 + (s_['z'] - zc) ** 2)
            ar_.plot(s_['t'], d - R, color=C[n], lw=1.0)
    a3.set_xlabel('x (m)')
    a3.set_ylabel('y (m)')
    a3.set_zlabel('z (m)')
    a3.set_title('paths, end positions and the quartic loop' if qi else 'paths, end positions and the potential sphere')
    if None not in (zc, R):
        az.axhspan(zc - R, zc + R, color='tab:green', alpha=0.1, lw=0, label='sphere z band')
    for key, lab in (('z_min', 'z_min (floor)'), ('z_max', 'z_max (ceiling)')):
        if zl.get(key) is not None:
            az.axhline(zl[key], color='r', ls=':', lw=1, label=lab)
    az.set_ylabel('altitude z (m)')
    az.set_title(f'altitude   (max climb {zl.get("max_vz", "?")} m/s, z_stretch {zl.get("z_stretch", 1)})')
    az.legend(fontsize=7, ncol=2)
    shade_window(az, D)
    ar_.axhline(0, color='k', lw=1)
    ar_.set_ylabel('|alpha1| (quartic)' if qi else 'distance to the sphere surface (m)')
    ar_.set_xlabel('time (s)')
    ar_.set_title('|alpha1|  (0 = on the curve)' if qi else '|p - c| - R  (0 = on the surface)')
    shade_window(ar_, D)
    fig.suptitle(f'{D.run.name}   3D', x=0.02, ha='left', fontsize=13)
    p = os.path.join(outdir, '05_3d.png')
    fig.savefig(p, dpi=110, bbox_inches='tight')
    plt.close(fig)
    return p


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('run', nargs='?', default='latest', help='run folder, logs folder, or `latest` (default)')
    ap.add_argument('--out', default=None, help='output folder (default <run>/plots)')
    ap.add_argument('--collide', type=float, default=0.15, help='centre distance counted as a collision (m)')
    ap.add_argument('--close', type=float, default=0.25, help='centre distance counted as a close approach (m)')
    ap.add_argument('--target-spacing', type=float, default=0.3, help='wanted neighbour spacing for the formation figure (m)')
    ap.add_argument('--no-open', action='store_true', help='only write the PNGs: do not open windows / do not wait')
    ap.add_argument('--open', action='store_true', help=argparse.SUPPRESS)   # old flag, now the default
    a = ap.parse_args()
    path = a.run
    if path == 'latest':
        runs = os.path.join(HERE, 'logs', 'runs')
        lat = os.path.join(runs, 'latest')
        path = lat if os.path.isdir(lat) else ar.newest_run(os.path.join(HERE, 'logs'))
    D = RunData(path, a.collide, a.close)
    out = a.out or os.path.join(D.run.dir, 'plots')
    try:
        os.makedirs(out, exist_ok=True)
        if not os.access(out, os.W_OK):
            raise PermissionError(out)
    except PermissionError:
        # the run folder is owned by root until `./dev_tmux.sh stop` fixes the ownership
        out = os.path.join(HERE, 'plots', D.run.name)
        os.makedirs(out, exist_ok=True)
        print(f'(run folder not writable, probably still owned by root: writing to {out}; '
              f'run ./dev_tmux.sh stop to fix the ownership)')
    res = D.results()
    made = [fig_summary(D, res, out), fig_trajectories(D, out, a.target_spacing), fig_separation(D, res, out),
            fig_sph(D, out), fig_performance(D, out), fig_3d(D, out)]
    made = [m for m in made if m]
    print(f'run {D.run.name}: {D.n} drones, window {res["window"][0]:.1f}..{res["window"][1]:.1f} s')
    if 'dmin' in res:
        print(f'  closest approach {res["dmin"]:.3f} m | collisions (<{a.collide:g} m): {len(res["ep_collide"])} '
              f'| close (<{a.close:g} m): {len(res["ep_close"])} | median NN {res["nn_med"]:.3f} m')
    if 'rtf_mean' in res:
        print(f'  RTF mean {res["rtf_mean"]:.2f}, min {res["rtf_min"]:.2f}')
    if 'busy' in res:
        print(f'  busy threads {res["busy"]:.2f} of {res["ncpu"]:.0f}, clock {res["clock"]:.0f} MHz')
    for m in made:
        print(f'  saved {m}')
    if made and not a.no_open:
        show_and_wait(made)


if __name__ == '__main__':
    main()