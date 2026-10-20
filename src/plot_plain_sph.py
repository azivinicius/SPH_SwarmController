#!/usr/bin/env python3
"""Plots of ONE run, shown in windows that stay open until you press Ctrl+C.

    python3 src/plot_plain_sph.py                      # newest run in src/logs/runs
    python3 src/plot_plain_sph.py src/logs/runs/<id>   # a given run (or `latest`)
    python3 src/plot_plain_sph.py <run> --no-open      # only write the PNGs, no windows, exit at once
    python3 src/plot_plain_sph.py <run> --out figs --collide 0.12 --target-spacing 0.3

The figures go to <run>/plots/ , ONE CHART PER FIGURE, titles / labels / legends in Portuguese with a decimal
comma (monograph style). --pdf also writes a vector PDF of each one (for LaTeX). They open together in ONE
viewer window (feh: arrow keys to browse; else eog / display; install with `sudo apt install feh`, or set
PLOT_VIEWER="cmd args"); Ctrl+C (or closing the window) ends the script.

    00_resumo                   parameter and result table
    01_trajetorias_xy           paths in the horizontal plane + target (circle / sphere outline / quartic) + obstacles
    02_trajetorias_3d           3D runs only: 3D paths + target sphere / loop (equal axis scales)
    03_altitude                 altitude of every drone (2D: reference altitude; 3D: sphere band, floor, ceiling)
    04_erro_alvo                distance to the target: |p - c| - R (circle / sphere) or alpha1 (quartic)
    05_formacao_1..4            formation snapshots (links: green = target spacing +-20 %, orange tighter, blue looser)
    06_distancia_minima         closest pair vs time, collision threshold and collision episodes
    07_vizinho_mais_proximo     nearest-neighbour histogram in the analysis window
    08_balanco_densidade        ring density vs spacing for this mass, h, N
    09_densidade .. 13_velocidade_comandada   rho/rho0, pressure, potential acceleration, measured / commanded speed
    14_rtf .. 17_temperatura_cpu              RTF, CPU load per group, CPU clock, CPU temperature (rtf.csv / sys.csv)

Distances are centre to centre in 3D (2D when there is no altitude). "Collision" = centre distance below
--collide (default 0.10 m). One episode = one stretch below the threshold; gaps shorter than 0.5 s are merged.
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
GROUP_LABEL = {'gz': 'Gazebo', 'cf2': 'Firmware (cf2)', 'server': 'crazyflie_server', 'rviz': 'RViz',
               'ctrl': 'Controlador SPH', 'tools': 'Registro de dados', 'ros': 'Outros ROS', 'other': 'Outros'}


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
    try:                                  # one window with every figure (feh / eog: arrow keys to browse)
        procs.append(subprocess.Popen(cmd + list(paths), start_new_session=True,
                                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
    except OSError as exc:
        print(f'(could not open the figures: {exc})')
    if not procs:
        return
    t0 = time.time()
    print(f'{len(paths)} figures open in {cmd[0]} (arrow keys to browse). Ctrl+C here closes it.')
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
    def __init__(self, path, collide, skip=2.0):
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
        self.collide = collide
        # ---- obstacles (params 'obstacles' = 'x,y/x,y,z/...'): [(x, y, z or None)]; z None = 2D, or a 3D pillar
        self.dim3 = int(float(self.P.get('dim', 2) or 2)) == 3
        self.obs = []
        for item in str(self.P.get('obstacles', 'none') or 'none').replace(' ', '').split('/'):
            if item and item.lower() != 'none':
                v = [float(s) for s in item.split(',')]
                self.obs.append((v[0], v[1], v[2] if len(v) == 3 else None))
        self.obs_reach = 2.0 * float(self.sph.get('h') or self.P.get('smoothing_h') or 0.3)   # obstacles push within 2h
        self.obs_dist = None              # closest drone-obstacle distance on self.grid (set by _pair_distances)
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
        if self.obs:
            # 2D run / 3D pillar: horizontal distance; 3D point obstacle: 3D distance
            od = []
            for p_ in pos.values():
                for ox, oy, oz in self.obs:
                    d2 = (p_[0] - ox) ** 2 + (p_[1] - oy) ** 2
                    if oz is not None and self.dim3 and p_.shape[0] == 3:
                        d2 = d2 + (p_[2] - oz) ** 2
                    od.append(np.sqrt(d2))
            self.obs_dist = np.vstack(od).min(axis=0)
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
                res['t_below_collide'] = float((dm < self.collide).sum() * 0.1)
                nn = np.concatenate([v[w] for v in self.nn.values()])
                res['nn_med'] = float(np.median(nn))
                res['nn_p5'], res['nn_p95'] = float(np.percentile(nn, 5)), float(np.percentile(nn, 95))
                if self.obs_dist is not None:
                    res['obs_dmin'] = float(self.obs_dist[w].min())
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
# One chart per figure, titles / axis labels / legends in Portuguese (monograph style), decimal comma.
import matplotlib.ticker as mticker          # noqa: E402
from matplotlib.lines import Line2D          # noqa: E402

plt.rcParams.update({
    'font.family': 'serif', 'mathtext.fontset': 'dejavuserif',
    'font.size': 11, 'axes.titlesize': 12, 'axes.labelsize': 11, 'legend.fontsize': 9,
    'xtick.labelsize': 10, 'ytick.labelsize': 10, 'savefig.dpi': 200,
    'axes.grid': True, 'grid.alpha': 0.3, 'legend.framealpha': 0.9,
})
FIG_W, FIG_H = 7.5, 4.6          # one chart, about the text width of an A4 page


class _VirgulaDecimal(mticker.ScalarFormatter):
    """Tick labels with a decimal comma (ABNT)."""
    def __call__(self, x, pos=None):
        return super().__call__(x, pos).replace('.', ',')


def num(v, nd=2):
    """Number as text with a decimal comma; 'n/d' when missing."""
    if v is None or (isinstance(v, float) and not math.isfinite(v)):
        return 'n/d'
    return f'{v:.{nd}f}'.replace('.', ',')


def drone_label(n):
    return 'Drone ' + n.replace('cf_', '')


def color_map(names):
    cyc = plt.rcParams['axes.prop_cycle'].by_key()['color']
    return {n: cyc[i % len(cyc)] for i, n in enumerate(names)}


def shade_window(ax, D, label=True):
    """Grey band = analysis window (all drones airborne, up to the end or a divergence)."""
    ax.axvspan(D.w0, D.w1, color='0.5', alpha=0.08, lw=0, label='Janela de análise' if label else None)


def new_fig(w=FIG_W, h=FIG_H, projection=None):
    fig = plt.figure(figsize=(w, h))
    ax = fig.add_subplot(111, projection=projection)
    return fig, ax


def save(fig, outdir, name, pdf=False):
    for ax in fig.axes:
        axes = [ax.xaxis, ax.yaxis] + ([ax.zaxis] if hasattr(ax, 'zaxis') else [])
        for a in axes:
            if isinstance(a.get_major_formatter(), mticker.ScalarFormatter):
                a.set_major_formatter(_VirgulaDecimal())
    p = os.path.join(outdir, name + '.png')
    fig.savefig(p, bbox_inches='tight')
    if pdf:
        fig.savefig(os.path.join(outdir, name + '.pdf'), bbox_inches='tight')
    plt.close(fig)
    return p


def legend_out(ax, **kw):
    """Legend to the right of the axes (keeps the data area free)."""
    return ax.legend(loc='upper left', bbox_to_anchor=(1.02, 1.0), borderaxespad=0.0, **kw)


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


def round_target(D):
    """(xc, yc, zc, R) of the circle (2D) / sphere (3D) target, or None (quartic, no potential)."""
    if str(D.P.get('potential', 'default')).lower() == 'quartic' or not D.P.get('f_k'):
        return None
    pot = D.core.get('potential') or {}
    xc, yc, R = pot.get('xc', D.P.get('xc')), pot.get('yc', D.P.get('yc')), pot.get('R', D.P.get('R'))
    if None in (xc, yc, R):
        return None
    zc = pot.get('zc', D.P.get('zc')) if D.dim3 else None
    return float(xc), float(yc), (float(zc) if zc is not None else None), float(R)


def draw_target_xy(ax, D, lw=1.6, label=True):
    """Target seen from above (circle, sphere outline or quartic curve) + its centre + obstacles."""
    rt, qi = round_target(D), quartic_info(D)
    if rt:
        xc, yc, _, R = rt
        th = np.linspace(0, 2 * math.pi, 400)
        lab = (f'Contorno da esfera-alvo, vista superior (R = {num(R, 3)} m)' if D.dim3
               else f'Circunferência-alvo (R = {num(R, 3)} m)')
        ax.plot(xc + R * np.cos(th), yc + R * np.sin(th), 'k--', lw=lw, label=lab if label else None)
        ax.plot(xc, yc, 'k+', ms=10, mew=1.5, label='Centro do potencial' if label else None)
    if qi:
        qx, qy = quartic_curve(qi)
        lab = (f'Projeção da curva-alvo quártica (R = {num(qi["R"], 3)})' if D.dim3
               else f'Curva-alvo quártica (R = {num(qi["R"], 3)})')
        ax.plot(qx, qy, 'k--', lw=lw, label=lab if label else None)
        ax.plot(qi['xc'], qi['yc'], 'k+', ms=10, mew=1.5, label='Centro do potencial' if label else None)
    for k, (ox, oy, oz) in enumerate(D.obs):
        lab = f'Obstáculo (alcance 2h = {num(D.obs_reach, 2)} m)' if (k == 0 and label) else None
        ax.plot(ox, oy, 'x', color='tab:red', ms=10, mew=2.5, zorder=4, label=lab)
        ax.add_patch(plt.Circle((ox, oy), D.obs_reach, fill=True, fc='tab:red', alpha=0.08, ec='tab:red',
                                ls='--', lw=1.0, zorder=0))


def xy_limits(D):
    xs, ys = [], []
    for n in D.names:
        s = D.S[n]
        if finite(s['x']):
            xs += list(s['x'][np.isfinite(s['x'])])
            ys += list(s['y'][np.isfinite(s['y'])])
    if not xs:
        return None
    rt, qi = round_target(D), quartic_info(D)
    if rt:
        xc, yc, _, R = rt
        xs += [xc - R, xc + R]
        ys += [yc - R, yc + R]
    if qi:
        qx, qy = quartic_curve(qi)
        xs += [float(qx.min()), float(qx.max())]
        ys += [float(qy.min()), float(qy.max())]
    for ox, oy, _ in D.obs:
        xs += [ox - D.obs_reach, ox + D.obs_reach]
        ys += [oy - D.obs_reach, oy + D.obs_reach]
    span = max(max(xs) - min(xs), max(ys) - min(ys), 1.0)
    mg = 0.08 * span
    return min(xs) - mg, max(xs) + mg, min(ys) - mg, max(ys) + mg


# ---- 00: parameters and results table ---------------------------------------------------------------
def fig_resumo(D, res, outdir, pdf=False):
    fig = plt.figure(figsize=(13, 8.5))
    ax = fig.add_axes([0.02, 0.02, 0.96, 0.9])
    ax.axis('off')
    ax.grid(False)
    P, S = D.P, D.sph
    r_eq, s_eq = D.equilibrium()

    def line(k, v):
        return f'{k:<34}{v}'

    tgt = str(P.get('potential', 'default')).lower()
    tgt = {'quartic': 'quártica', 'default': 'esfera' if D.dim3 else 'circunferência',
           'circle': 'circunferência', 'sphere': 'esfera'}.get(tgt, tgt)
    left = [
        'ENSAIO', line('Identificação', D.run.name), line('Rótulo', str(D.meta.get('label', ''))),
        line('Versão do código', str(D.meta.get('git_rev', '?'))),
        line('Número de drones', D.n), line('Dimensão do controle', '3D' if D.dim3 else '2D'),
        line('Plataforma / relógio', f'{P.get("platform", "sim")} / {D.core.get("sim_clock", "wall")}'),
        '',
        'PARÂMETROS SPH', line('Massa m', num(float(S.get('mass', P.get('particle_mass', float('nan')))), 3)),
        line('Comprimento de suavização h', f'{num(float(S.get("h", P.get("smoothing_h", float("nan")))), 3)} m'),
        line('Densidade de referência ρ0', S.get('rho_0', P.get('rho_0', '?'))),
        line('Expoente γ', S.get('gamma', P.get('gamma', '?'))),
        line('Viscosidade artificial α / β', f'{P.get("alpha", "?")} / {P.get("beta", "?")}'),
        line('Amortecimento ζ', S.get('zetta_extra', P.get('zetta_extra', '?'))),
        line('Ganho SPH', P.get('sph_gain', '?')),
        line('Espaçamento de equilíbrio (anel)', f'{num(s_eq, 3)} m'),
        '',
        'CONTROLE E POTENCIAL', line('Período de controle', f'{num(D.period * 1000, 0)} ms'),
        line('Velocidade máxima', f'{num(float(D.max_v), 2)} m/s'),
        line('Formação-alvo', tgt),
        line('Ganho do potencial k', P.get('f_k', '?')),
        line('Ganho tangencial', P.get('f_kt', '0')),
        line('Centro e raio (xc, yc, R)', f'{P.get("xc", "?")}, {P.get("yc", "?")}, {P.get("R", "?")}'),
        line('Altitude de referência', f'{num(float(D.fixed_z), 2)} m' if not D.dim3 else 'livre (3D)'),
        line('Obstáculos', f'{len(D.obs)}' if D.obs else 'nenhum'),
    ]
    w0, w1 = res['window']
    right = ['RESULTADOS (janela de análise)',
             line('Janela de análise', f'{num(w0, 1)} a {num(w1, 1)} s' + ('' if D.flight_known else ' (ensaio inteiro)'))]
    if D.diverged:
        right.append(line('Divergência detectada', D.diverged_why))
    right += ['', 'SEPARAÇÃO ENTRE DRONES']
    if 'dmin' in res:
        right += [line('Menor distância entre pares', f'{num(res["dmin"], 3)} m'),
                  line('Mediana da menor distância', f'{num(res["dmin_med"], 3)} m'),
                  line('Vizinho mais próximo (mediana)', f'{num(res["nn_med"], 3)} m'),
                  line('Vizinho mais próximo (p5 / p95)', f'{num(res["nn_p5"], 3)} / {num(res["nn_p95"], 3)} m'),
                  line(f'Colisões (< {num(D.collide, 2)} m)', f'{len(res["ep_collide"])} episódio(s), {num(res["t_below_collide"], 1)} s')]
        if 'obs_dmin' in res:
            right.append(line('Menor distância a obstáculo', f'{num(res["obs_dmin"], 3)} m'))
    else:
        right.append('sem dados de posição')
    right += ['', 'MALHA DE CONTROLE E SIMULADOR',
              line('RTF médio / mínimo', f'{num(res.get("rtf_mean"))} / {num(res.get("rtf_min"))}'),
              line('Tempo de cálculo (p95)', f'{num(res.get("step_p95"), 2)} ms'),
              line('Ciclos atrasados (> 1,5 período)', f'{num(res.get("late_pct"), 1)} %'),
              line('Ciclos sem dado novo', f'{num(res.get("stale_pct"), 1)} %'),
              line('Comando saturado', f'{num(res.get("clip_pct"), 1)} %')]
    if 'busy' in res:
        right += ['', 'COMPUTADOR',
                  line('Núcleos ocupados (média)', f'{num(res["busy"], 2)} de {res["ncpu"]:.0f}'),
                  line('Frequência / temperatura máx.', f'{num(res["clock"], 0)} MHz / {num(res["temp"], 0)} °C')]
    ax.text(0.0, 1.0, '\n'.join(left), va='top', ha='left', family='monospace', fontsize=9.5, transform=ax.transAxes)
    ax.text(0.52, 1.0, '\n'.join(right), va='top', ha='left', family='monospace', fontsize=9.5, transform=ax.transAxes)
    fig.suptitle('Resumo dos parâmetros e resultados do ensaio', fontsize=13, x=0.02, ha='left')
    p = os.path.join(outdir, '00_resumo.png')
    fig.savefig(p, dpi=150)
    if pdf:
        fig.savefig(os.path.join(outdir, '00_resumo.pdf'))
    plt.close(fig)
    return p


# ---- 01: paths in the horizontal plane ------------------------------------------------------------
def fig_trajetorias_xy(D, outdir, pdf=False):
    lim = xy_limits(D)
    if lim is None:
        return None
    C = color_map(D.names)
    fig, ax = new_fig(8.5, 6.5)
    for n in D.names:
        s = D.S[n]
        if not finite(s['x']):
            continue
        t, x, y = s['t'], s['x'], s['y']
        w = (t >= D.w0) & (t <= D.w1)
        ax.plot(x, y, '-', color=C[n], lw=0.8, alpha=0.3)
        if w.any():
            ax.plot(x[w], y[w], '-', color=C[n], lw=1.6, label=drone_label(n))
            ax.plot(x[w][0], y[w][0], 'o', color=C[n], mec='k', ms=6)
            ax.plot(x[w][-1], y[w][-1], 's', color=C[n], mec='k', ms=7)
    draw_target_xy(ax, D)
    h, l = ax.get_legend_handles_labels()
    h += [Line2D([], [], ls='', marker='o', color='0.6', mec='k', label='Posição inicial'),
          Line2D([], [], ls='', marker='s', color='0.6', mec='k', label='Posição final'),
          Line2D([], [], color='0.6', lw=0.8, alpha=0.5, label='Decolagem e pouso')]
    ax.legend(handles=h, loc='upper left', bbox_to_anchor=(1.02, 1.0), borderaxespad=0.0)
    ax.set_xlim(lim[0], lim[1])
    ax.set_ylim(lim[2], lim[3])
    ax.set_aspect('equal')
    ax.set_xlabel('Posição x (m)')
    ax.set_ylabel('Posição y (m)')
    ax.set_title('Trajetórias dos drones no plano horizontal')
    return save(fig, outdir, '01_trajetorias_xy', pdf)


# ---- 02: 3D paths (3D runs) ------------------------------------------------------------------------
def fig_trajetorias_3d(D, outdir, pdf=False):
    if not D.dim3:
        return None
    ok = [n for n in D.names if finite(D.S[n]['z']) and finite(D.S[n]['x'])]
    if not ok:
        return None
    C = color_map(D.names)
    fig, ax = new_fig(8.5, 7.0, projection='3d')
    rt, qi = round_target(D), quartic_info(D)
    extra = []
    if rt and rt[2] is not None:
        xc, yc, zc, R = rt
        u, v = np.meshgrid(np.linspace(0, 2 * math.pi, 40), np.linspace(0, math.pi, 20))
        ax.plot_wireframe(xc + R * np.cos(u) * np.sin(v), yc + R * np.sin(u) * np.sin(v), zc + R * np.cos(v),
                          color='0.6', lw=0.4, alpha=0.6)
        extra.append(Line2D([], [], color='0.6', lw=1, label=f'Esfera-alvo (R = {num(R, 3)} m)'))
    if qi:
        qx, qy = quartic_curve(qi)
        ax.plot(qx, qy, (qx + qy) / 2.0, 'k--', lw=1.6)        # the loop lies on the plane z = (x + y) / 2
        extra.append(Line2D([], [], color='k', ls='--', lw=1.6, label=f'Curva-alvo quártica (R = {num(qi["R"], 3)})'))
    zl = D.core.get('z_limits') or {}
    z_lo, z_hi = zl.get('z_min', 0.25), zl.get('z_max', 2.5)
    for k, (ox, oy, oz) in enumerate(D.obs):
        lab = 'Obstáculo' if k == 0 else None
        if oz is None:
            ax.plot([ox, ox], [oy, oy], [z_lo, z_hi], color='tab:red', lw=4, alpha=0.8, label=lab)
        else:
            ax.scatter(ox, oy, oz, color='tab:red', marker='X', s=120, edgecolor='k', label=lab)
    pts = []
    for n in ok:
        s_ = D.S[n]
        ax.plot(s_['x'], s_['y'], s_['z'], color=C[n], lw=1.0, alpha=0.8, label=drone_label(n))
        ax.scatter(s_['x'][-1], s_['y'][-1], s_['z'][-1], color=C[n], s=40, edgecolor='k')
        pts.append(np.column_stack([s_['x'], s_['y'], s_['z']]))
    # equal scale on the three axes, otherwise the sphere is drawn as an ellipsoid
    pts = np.concatenate(pts)
    if rt and rt[2] is not None:
        pts = np.vstack([pts, [[xc - R, yc - R, zc - R], [xc + R, yc + R, zc + R]]])
    pts = pts[np.isfinite(pts).all(axis=1)]
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    mid, half = (lo + hi) / 2.0, max(float((hi - lo).max()) / 2.0, 0.5) * 1.05
    ax.set_xlim(mid[0] - half, mid[0] + half)
    ax.set_ylim(mid[1] - half, mid[1] + half)
    z0 = max(mid[2] - half, 0.0)
    ax.set_zlim(z0, z0 + 2 * half)
    ax.set_box_aspect((1, 1, 1))
    ax.set_xlabel('Posição x (m)')
    ax.set_ylabel('Posição y (m)')
    ax.set_zlabel('Altitude z (m)')
    h, _ = ax.get_legend_handles_labels()
    h += extra + [Line2D([], [], ls='', marker='o', color='0.6', mec='k', label='Posição final')]
    ax.legend(handles=h, loc='upper left', bbox_to_anchor=(1.05, 1.0), borderaxespad=0.0)
    ax.set_title('Trajetórias tridimensionais dos drones')
    return save(fig, outdir, '02_trajetorias_3d', pdf)


# ---- 03: altitude -----------------------------------------------------------------------------------
def fig_altitude(D, outdir, pdf=False):
    ok = [n for n in D.names if finite(D.S[n]['z'])]
    if not ok:
        return None
    C = color_map(D.names)
    fig, ax = new_fig()
    for n in ok:
        ax.plot(D.S[n]['t'], D.S[n]['z'], color=C[n], lw=1.1, label=drone_label(n))
    if D.dim3:
        rt = round_target(D)
        if rt and rt[2] is not None:
            ax.axhspan(rt[2] - rt[3], rt[2] + rt[3], color='tab:green', alpha=0.1, lw=0,
                       label='Faixa de altitude da esfera-alvo')
        zl = D.core.get('z_limits') or {}
        if zl.get('z_min') is not None:
            ax.axhline(zl['z_min'], color='r', ls=':', lw=1, label=f'Altitude mínima ({num(float(zl["z_min"]), 2)} m)')
        if zl.get('z_max') is not None:
            ax.axhline(zl['z_max'], color='r', ls='-.', lw=1, label=f'Altitude máxima ({num(float(zl["z_max"]), 2)} m)')
    else:
        ax.axhline(D.fixed_z, color='k', ls=':', lw=1.2, label=f'Altitude de referência ({num(float(D.fixed_z), 2)} m)')
    shade_window(ax, D)
    ax.set_xlabel('Tempo (s)')
    ax.set_ylabel('Altitude z (m)')
    ax.set_title('Altitude dos drones ao longo do ensaio')
    legend_out(ax)
    return save(fig, outdir, '03_altitude', pdf)


# ---- 04: distance to the target shape ----------------------------------------------------------------
def fig_erro_alvo(D, outdir, pdf=False):
    rt, qi = round_target(D), quartic_info(D)
    if not (rt or qi):
        return None
    C = color_map(D.names)
    fig, ax = new_fig()
    drawn = False
    for n in D.names:
        s = D.S[n]
        if not finite(s['x']):
            continue
        if qi:
            e = quartic_alpha1(qi, s['x'], s['y'])
        elif D.dim3 and rt[2] is not None and finite(s['z']):
            e = np.sqrt((s['x'] - rt[0]) ** 2 + (s['y'] - rt[1]) ** 2 + (s['z'] - rt[2]) ** 2) - rt[3]
        else:
            e = np.hypot(s['x'] - rt[0], s['y'] - rt[1]) - rt[3]
        ax.plot(s['t'], e, color=C[n], lw=1.1, label=drone_label(n))
        drawn = True
    if not drawn:
        plt.close(fig)
        return None
    ax.axhline(0, color='k', lw=1.0, label='Sobre a formação-alvo')
    shade_window(ax, D)
    ax.set_xlabel('Tempo (s)')
    if qi:
        ax.set_ylabel(r'Função de nível $\alpha_1$')
        ax.set_title(r'Convergência à curva-alvo quártica ($\alpha_1 = 0$ sobre a curva, $\alpha_1 < 0$ no interior)')
    else:
        ax.set_ylabel(r'$\|\mathbf{p}-\mathbf{c}\| - R$ (m)')
        ax.set_title('Distância de cada drone à ' + ('superfície da esfera-alvo' if D.dim3 else 'circunferência-alvo'))
    legend_out(ax)
    return save(fig, outdir, '04_erro_alvo', pdf)


# ---- 05: formation snapshots, one figure each -------------------------------------------------------
def fig_formacao(D, outdir, target=0.3, nsnap=4, pdf=False):
    lim = xy_limits(D)
    ok = [n for n in D.names if finite(D.S[n]['x']) and finite(D.S[n]['y'])]
    if lim is None or not ok or D.grid is None:
        return []
    C = color_map(D.names)
    g = D.grid
    gm = (g >= D.w0) & (g <= D.w1)
    g = g[gm] if gm.sum() > nsnap else g
    X = np.vstack([np.interp(g, D.S[n]['t'], D.S[n]['x']) for n in ok])
    Y = np.vstack([np.interp(g, D.S[n]['t'], D.S[n]['y']) for n in ok])
    use_z = D.dim3 and all(finite(D.S[n]['z']) for n in ok)     # 3D: links use the real 3D distance
    Z = np.vstack([np.interp(g, D.S[n]['t'], D.S[n]['z']) for n in ok]) if use_z else np.zeros_like(X)
    idx = np.unique(np.linspace(0, len(g) - 1, nsnap).astype(int))
    out = []
    for k, i in enumerate(idx):
        fig, ax = new_fig(7.5, 6.0)
        draw_target_xy(ax, D, lw=1.0)
        for a in range(len(ok)):
            for b in range(a + 1, len(ok)):
                d = math.sqrt((X[a, i] - X[b, i]) ** 2 + (Y[a, i] - Y[b, i]) ** 2 + (Z[a, i] - Z[b, i]) ** 2)
                if d < 1.6 * target:
                    col = 'tab:green' if abs(d - target) <= 0.2 * target else ('tab:orange' if d < target else 'tab:blue')
                    ax.plot([X[a, i], X[b, i]], [Y[a, i], Y[b, i]], color=col, lw=1.6, alpha=0.8, zorder=1)
        for a, n in enumerate(ok):
            ax.scatter(X[a, i], Y[a, i], s=60, color=C[n], edgecolor='k', zorder=3)
            ax.annotate(n.replace('cf_', ''), (X[a, i], Y[a, i]), xytext=(4, 4), textcoords='offset points', fontsize=9)
        h, _ = ax.get_legend_handles_labels()
        dist = 'distância 3D' if use_z else 'distância'
        h += [Line2D([], [], ls='', marker='o', color='0.6', mec='k', label='Drone (número = identificador)'),
              Line2D([], [], color='tab:green', lw=1.6, label=f'Vizinhos a {num(target, 2)} m ± 20 % ({dist})'),
              Line2D([], [], color='tab:orange', lw=1.6, label='Vizinhos mais próximos que o alvo'),
              Line2D([], [], color='tab:blue', lw=1.6, label=f'Vizinhos mais afastados (até {num(1.6 * target, 2)} m)')]
        ax.legend(handles=h, loc='upper left', bbox_to_anchor=(1.02, 1.0), borderaxespad=0.0)
        ax.set_xlim(lim[0], lim[1])
        ax.set_ylim(lim[2], lim[3])
        ax.set_aspect('equal')
        ax.set_xlabel('Posição x (m)')
        ax.set_ylabel('Posição y (m)')
        ax.set_title(f'Formação dos drones no instante t = {num(float(g[i]), 1)} s'
                     + (' (vista superior)' if D.dim3 else ''))
        out.append(save(fig, outdir, f'05_formacao_{k + 1}', pdf))
    return out


# ---- 06 - 08: separation --------------------------------------------------------------------------
def fig_distancia_minima(D, res, outdir, pdf=False):
    if D.grid is None:
        return None
    r_eq, s_eq = D.equilibrium()
    fig, ax = new_fig()
    ax.plot(D.grid, D.dmin, 'k', lw=1.4, label='Menor distância entre pares de drones')
    if D.obs_dist is not None:
        ax.plot(D.grid, D.obs_dist, color='darkred', ls='-.', lw=1.4, label='Menor distância drone–obstáculo')
        ax.axhline(D.obs_reach, color='darkred', ls=':', lw=1, label=f'Alcance do obstáculo ({num(D.obs_reach, 2)} m)')
    ax.axhline(D.collide, color='r', ls='-', lw=1.2, label=f'Limiar de colisão ({num(D.collide, 2)} m)')
    if math.isfinite(s_eq):
        ax.axhline(s_eq, color='g', ls=':', lw=1.4, label=f'Espaçamento de equilíbrio SPH ({num(s_eq, 2)} m)')
    if 'ep_collide' in res:
        for k, (t0, t1) in enumerate(res['ep_collide']):
            ax.axvspan(t0 - 0.05, t1 + 0.05, color='r', alpha=0.25, lw=0, label='Episódio de colisão' if k == 0 else None)
    shade_window(ax, D)
    ax.set_ylim(0, max(1.0, min(float(np.nanpercentile(D.dmin, 99)) * 1.4, 6.0)))
    ax.set_xlabel('Tempo (s)')
    ax.set_ylabel(f'Distância entre centros, {"3D" if getattr(D, "dims3", False) else "no plano"} (m)')
    ax.set_title('Menor distância entre drones ao longo do ensaio')
    legend_out(ax)
    return save(fig, outdir, '06_distancia_minima', pdf)


def fig_vizinho(D, res, outdir, pdf=False):
    if D.grid is None:
        return None
    w = D.gmask()
    if not w.any():
        return None
    r_eq, s_eq = D.equilibrium()
    nn = np.concatenate([v[w] for v in D.nn.values()])
    hi = max(float(np.nanpercentile(nn, 99)) * 1.1, D.collide * 3.0)
    fig, ax = new_fig()
    ax.hist(nn, bins=np.linspace(0, hi, 40), color='0.55', edgecolor='0.3', lw=0.4,
            weights=np.full(len(nn), 100.0 / len(nn)), label='Amostras (todos os drones)')
    ax.axvline(D.collide, color='r', lw=1.2, label=f'Limiar de colisão ({num(D.collide, 2)} m)')
    if math.isfinite(s_eq):
        ax.axvline(s_eq, color='g', ls=':', lw=1.4, label=f'Espaçamento de equilíbrio SPH ({num(s_eq, 2)} m)')
    ax.axvline(float(np.median(nn)), color='b', lw=1.2, label=f'Mediana ({num(float(np.median(nn)), 2)} m)')
    ax.set_xlabel('Distância ao vizinho mais próximo (m)')
    ax.set_ylabel('Frequência relativa (%)')
    ax.set_title('Distribuição da distância ao vizinho mais próximo (janela de análise)')
    legend_out(ax)
    return save(fig, outdir, '07_vizinho_mais_proximo', pdf)


def fig_balanco_densidade(D, res, outdir, pdf=False):
    m, h, rho0 = D.sph.get('mass'), D.sph.get('h'), D.sph.get('rho_0') or 1000.0
    if m is None or h is None or D.n < 2:
        return None
    r_eq, s_eq = D.equilibrium()
    sp = np.linspace(0.05, max(3.0 * h, 1.5 * (s_eq if math.isfinite(s_eq) else 1.0), 1.0), 300)
    dens = np.array([ar.ring_density(D.n, s / (2 * math.sin(math.pi / D.n)), m, h) for s in sp])
    fig, ax = new_fig()
    ax.plot(sp, dens / rho0, 'k', lw=1.5,
            label=f'Densidade num anel uniforme (N = {D.n}, m = {num(float(m), 1)}, h = {num(float(h), 2)} m)')
    ax.axhline(1.0, color='g', lw=1, ls='--', label=r'$\rho = \rho_0$ (pressão nula)')
    if math.isfinite(s_eq):
        ax.axvline(s_eq, color='g', ls=':', lw=1.2, label=f'Espaçamento de equilíbrio ({num(s_eq, 2)} m)')
    if 'nn_med' in res:
        ax.axvline(res['nn_med'], color='b', lw=1.2, label=f'Mediana medida do vizinho mais próximo ({num(res["nn_med"], 2)} m)')
    ax.axvspan(0, D.collide, color='r', alpha=0.12, label=f'Região de colisão (< {num(D.collide, 2)} m)')
    ax.set_ylim(0, 4)
    ax.set_xlabel('Espaçamento entre vizinhos (m)')
    ax.set_ylabel(r'Densidade normalizada $\rho/\rho_0$')
    ax.set_title(r'Densidade SPH em função do espaçamento ($\rho > \rho_0$: repulsão; $\rho < \rho_0$: atração)')
    legend_out(ax)
    return save(fig, outdir, '08_balanco_densidade', pdf)


# ---- 09 - 13: SPH dynamics --------------------------------------------------------------------------
def _norm_of(T, k):
    comps = [np.asarray(T[f'{k}_{c}'], dtype=float) for c in 'xyz' if f'{k}_{c}' in T]
    comps = [c for c in comps if finite(c)]            # the z column is empty in 2D runs
    return np.sqrt(sum(c ** 2 for c in comps)) if comps else None


def _per_drone(D, outdir, name, title, ylabel, series, extra=None, pdf=False, mean=False):
    """One line per drone: series(T, t) -> y or None; extra(ax) adds reference lines.
    mean=True: drone lines translucent + the swarm mean (interpolated on D.grid) as a thick black line."""
    C = color_map(D.names)
    fig, ax = new_fig()
    drawn = False
    on_grid = []
    for n in D.names:
        T, t = D.R.ticks[n], D.S[n]['t']
        y = series(T, t)
        if y is None or len(y) != len(t) or not finite(y):
            continue
        ax.plot(t, y, color=C[n], lw=1.0, alpha=0.35 if mean else 1.0, label=drone_label(n))
        if mean and D.grid is not None:
            ok = np.isfinite(y)
            on_grid.append(np.interp(D.grid, np.asarray(t)[ok], np.asarray(y)[ok], left=np.nan, right=np.nan))
        drawn = True
    if not drawn:
        plt.close(fig)
        return None
    if on_grid:
        with np.errstate(all='ignore'):
            ym = np.nanmean(np.vstack(on_grid), axis=0)
        ax.plot(D.grid, ym, color='red', lw=2.2, label='Média do enxame')
    if extra:
        extra(ax)
    shade_window(ax, D)
    ax.set_xlabel('Tempo (s)')
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    legend_out(ax)
    return save(fig, outdir, name, pdf)


def figs_sph(D, outdir, pdf=False):
    rho0 = D.sph.get('rho_0') or 1000.0
    f = lambda v: np.asarray(v, dtype=float)

    def ref_rho(ax):
        ax.axhspan(0.9, 1.1, color='g', alpha=0.1, lw=0, label=r'Faixa de $\pm$10 % em torno de $\rho_0$')
        ax.axhline(1.0, color='g', lw=0.8, ls='--', label=r'$\rho = \rho_0$')

    def ref_zero(ax):
        ax.axhline(0, color='k', lw=0.8, label='Pressão nula')

    def ref_vmax(ax):
        ax.axhline(D.max_v, color='r', ls=':', lw=1.2, label=f'Velocidade máxima ({num(float(D.max_v), 2)} m/s)')

    return [
        _per_drone(D, outdir, '09_densidade', 'Densidade SPH normalizada de cada drone',
                   r'Densidade normalizada $\rho/\rho_0$',
                   lambda T, t: f(T['rho']) / rho0 if 'rho' in T else None, ref_rho, pdf, mean=True),
        _per_drone(D, outdir, '10_pressao', 'Pressão SPH de cada drone',
                   'Pressão P (unidades do modelo)',
                   lambda T, t: f(T['P']) if 'P' in T else None, ref_zero, pdf),
        _per_drone(D, outdir, '11_forca_potencial', 'Módulo da aceleração imposta pelo potencial de formação',
                   r'Aceleração do potencial (m/s$^2$)',
                   lambda T, t: _norm_of(T, 'ae'), None, pdf),
        _per_drone(D, outdir, '12_velocidade_medida', 'Velocidade horizontal medida de cada drone',
                   'Velocidade (m/s)',
                   lambda T, t: np.hypot(f(T['meas_vx']), f(T['meas_vy'])) if 'meas_vx' in T else None, ref_vmax, pdf),
        _per_drone(D, outdir, '13_velocidade_comandada', 'Velocidade horizontal comandada (antes da saturação)',
                   'Velocidade (m/s)',
                   lambda T, t: np.hypot(f(T['cmd_raw_x']), f(T['cmd_raw_y'])) if 'cmd_raw_x' in T else None,
                   ref_vmax, pdf),
    ]


# ---- 14 - 17: simulator and computer -----------------------------------------------------------------
def figs_desempenho(D, outdir, pdf=False):
    out = []
    if D.rtf_t is not None:
        fig, ax = new_fig()
        ax.plot(D.rtf_t, D.rtf_v, 'k', lw=1, label='RTF medido')
        ax.axhline(1.0, color='g', lw=0.8, ls='--', label='Tempo real (RTF = 1)')
        ax.axhline(0.9, color='orange', lw=0.8, ls=':', label='RTF = 0,9')
        ax.axhline(0.6, color='r', lw=0.8, ls=':', label='RTF = 0,6')
        ax.set_ylim(0, max(1.15, float(np.nanmax(D.rtf_v)) * 1.1))
        shade_window(ax, D)
        ax.set_xlabel('Tempo (s)')
        ax.set_ylabel('Fator de tempo real (RTF)')
        ax.set_title('Fator de tempo real da simulação')
        legend_out(ax)
        out.append(save(fig, outdir, '14_rtf', pdf))
    if D.sys is not None and D.sys_t is not None:
        s, t = D.sys, D.sys_t
        fig, ax = new_fig()
        ys = [np.asarray(s['cpu_' + g]) for g in GROUPS]
        keep = [k for k, y in enumerate(ys) if float(np.nanmax(y)) > 0.02]
        ax.stackplot(t, [ys[k] for k in keep], labels=[GROUP_LABEL[GROUPS[k]] for k in keep], alpha=0.85)
        ax.plot(t, np.asarray(s['cores_total']), 'k', lw=1.2, label='Total do computador')
        ax.axhline(float(s['ncpu'][0]), color='r', ls='--', lw=1, label=f'{s["ncpu"][0]:.0f} núcleos lógicos')
        ax.axhline(float(s['ncore'][0]), color='r', ls=':', lw=1, label=f'{s["ncore"][0]:.0f} núcleos físicos')
        ax.set_ylim(0, float(s['ncpu'][0]) * 1.5)
        shade_window(ax, D)
        ax.set_xlabel('Tempo (s)')
        ax.set_ylabel('Núcleos ocupados')
        ax.set_title('Ocupação da CPU por grupo de processos')
        legend_out(ax)
        out.append(save(fig, outdir, '15_carga_cpu', pdf))
        for name, col, title, ylabel, lab in (
                ('16_frequencia_cpu', 'freq_mhz', 'Frequência média de relógio da CPU', 'Frequência (MHz)', 'Frequência média'),
                ('17_temperatura_cpu', 'temp_c', 'Temperatura máxima da CPU', 'Temperatura (°C)', 'Zona mais quente')):
            fig, ax = new_fig()
            ax.plot(t, np.asarray(s[col]), 'b' if col == 'freq_mhz' else 'r', lw=1, label=lab)
            shade_window(ax, D)
            ax.set_xlabel('Tempo (s)')
            ax.set_ylabel(ylabel)
            ax.set_title(title)
            legend_out(ax)
            out.append(save(fig, outdir, name, pdf))
    return out


# ----------------------------------------------------------------------------- theory figures (--teoria)
# Figures for the methodology chapter, independent of any run: the cubic-spline kernel (2D and 3D) and the
# quartic potential (2D vector field, alpha1 level set, the two 3D surfaces whose intersection is the loop).
# The potential figures use the controller's own classes (potential.Potential / potential3d.Potential3D), so
# they show exactly the force the drones receive.  Change the values below to match the runs in the text.
TEORIA = dict(
    xc=2.0, yc=2.0,     # centre of the quartic (POT_XC, POT_YC)
    R=0.2,              # quartic size constant (POT_R)
    k=40.0,             # POT_K: the controller uses gain = k / 2
    kt=5.0,             # POT_KT for the field WITH the tangential term (0 = skip that figure)
    R_circ=1.0,         # circle potential figures: radius (dimensionless)
    k_circ=1.0,         # circle potential figures: gain k
    obs_h=0.3,          # obstacle figure: SPH_H (m)
    obs_m=70.0,         # obstacle figure: agent mass (SPH_MASS)
    obs_rho0=1000.0,    # rho_0
    obs_gamma=2.0,      # gamma of the equation of state
    obs_lambda=3.0,     # lambda = obs_sigma: rho_d >= lambda * rho_0
    obs_pts=[(0.9, 0.9)],               # one virtual particle per obstacle
    obs_radius=0.2,                     # drawn body of each obstacle (m)
    obs_view=(0.0, 2.8, 0.0, 2.8),      # window x0, x1, y0, y1 (potential in the upper right)
)   
SIGMA = {2: 10.0 / (7.0 * math.pi), 3: 1.0 / math.pi}      # sigma_d of W = sigma_d / h^d * f(q)


def _spline_f(q):
    """f(q) and f'(q) of the cubic spline, q = r / h (same branches as particle.W / grad_W)."""
    q = np.asarray(q, dtype=float)
    f = np.where(q <= 1, 1 - 1.5 * q ** 2 + 0.75 * q ** 3, np.where(q <= 2, 0.25 * (2 - q) ** 3, 0.0))
    df = np.where(q <= 1, -3 * q + 2.25 * q ** 2, np.where(q <= 2, -0.75 * (2 - q) ** 2, 0.0))
    return f, df


def _potential_classes():
    sys.path.insert(0, os.path.join(HERE, 'drone_controller'))
    from drone_controller.potential import Potential          # noqa: E402
    from drone_controller.potential3d import Potential3D      # noqa: E402
    return Potential, Potential3D


def teoria_nucleo_perfil(outdir, dim, pdf=False):
    q = np.linspace(0, 2.5, 600)
    f, df = _spline_f(q)
    s = SIGMA[dim]
    fig, ax = new_fig()
    ax.plot(q, s * f, 'k', lw=1.8, label=rf'Núcleo, $h^{dim}\,W(q)$')
    ax.plot(q, s * df, 'tab:blue', ls='--', lw=1.6, label=rf'Derivada, $h^{dim + 1}\,\partial W/\partial r$')
    ax.axvline(1.0, color='0.5', ls=':', lw=1.0, label='Mudança de ramo ($q = 1$)')
    ax.axvline(2.0, color='tab:red', ls='-.', lw=1.2, label='Limite do suporte compacto ($q = 2$, $r = 2h$)')
    ax.axhline(0, color='k', lw=0.6)
    ax.set_xlim(0, 2.5)
    ax.set_xlabel(r'Distância normalizada $q = r/h$')
    ax.set_ylabel('Valor adimensional')
    nome = 'bidimensional' if dim == 2 else 'tridimensional'
    ax.set_title(rf'Núcleo spline cúbico {nome} ($\sigma_{dim} = $' +
                 (r'$10/7\pi$)' if dim == 2 else r'$1/\pi$)'))
    legend_out(ax)
    return save(fig, outdir, f'teoria_01_nucleo_{dim}d_perfil' if dim == 2 else 'teoria_03_nucleo_3d_perfil', pdf)


def teoria_nucleo_2d_superficie(outdir, pdf=False):
    g = np.linspace(-2.2, 2.2, 161)
    X, Y = np.meshgrid(g, g)
    Wn = SIGMA[2] * _spline_f(np.hypot(X, Y))[0]
    fig, ax = new_fig(8.0, 6.0, projection='3d')
    surf = ax.plot_surface(X, Y, Wn, cmap='viridis', lw=0, antialiased=True, rstride=2, cstride=2)
    cb = fig.colorbar(surf, ax=ax, shrink=0.6, pad=0.1)
    cb.set_label(r'$W$')
    cb.formatter = _VirgulaDecimal()
    cb.update_ticks()
    ax.set_xlabel(r'$x$')
    ax.set_ylabel(r'$y$')
    ax.set_zlabel(r'$W(x, y)$')
    ax.set_title(r'Superfície do núcleo spline cúbico bidimensional ($h = 1$)')
    return save(fig, outdir, 'teoria_02_nucleo_2d_superficie', pdf)


def teoria_nucleo_3d_nuvem(outdir, pdf=False):
    """3D kernel as a point cloud: points on a regular grid inside the support r <= 2h, coloured by h^3 W.
    One octant is removed (cut-away) so the core, where W is largest, is visible."""
    g = np.arange(-2.0, 2.0001, 0.16)
    X, Y, Z = [a.ravel() for a in np.meshgrid(g, g, g, indexing='ij')]
    r = np.sqrt(X ** 2 + Y ** 2 + Z ** 2)
    keep = (r <= 2.0) & ~((X > 0) & (Y < 0) & (Z > 0))           # support ball minus one octant
    X, Y, Z, r = X[keep], Y[keep], Z[keep], r[keep]
    Wn = SIGMA[3] * _spline_f(r)[0]
    order = np.argsort(Wn)                                          # strongest points drawn last (on top)
    X, Y, Z, Wn = X[order], Y[order], Z[order], Wn[order]
    fig, ax = new_fig(8.0, 6.8, projection='3d')
    w = Wn / Wn.max()
    rgba = plt.get_cmap('viridis')(w)
    rgba[:, 3] = 0.35 + 0.65 * w                                   # weak values partly transparent
    ax.scatter(X, Y, Z, c=rgba, s=6 + 40 * w, lw=0, depthshade=False)
    sc = plt.cm.ScalarMappable(cmap='viridis', norm=plt.Normalize(0.0, float(Wn.max())))
    th = np.linspace(0, 2 * math.pi, 200)
    ax.plot(2 * np.cos(th), 2 * np.sin(th), 0 * th, color='tab:red', ls='-.', lw=1.2)
    cb = fig.colorbar(sc, ax=ax, shrink=0.6, pad=0.1)
    cb.set_label(r'$W$')
    cb.formatter = _VirgulaDecimal()
    cb.update_ticks()
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    ax.set_zlim(-2, 2)
    for setter in (ax.set_xticks, ax.set_yticks, ax.set_zticks):
        setter([-2, -1, 0, 1, 2])
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=22, azim=-60)
    ax.set_xlabel(r'$x$')
    ax.set_ylabel(r'$y$')
    ax.set_zlabel(r'$z$')
    ax.legend(handles=[Line2D([], [], color='tab:red', ls='-.', lw=1.2, label=r'Suporte compacto ($r = 2h$), plano $z = 0$'),
                       Line2D([], [], ls='', marker='o', color='0.5', ms=4,
                              label='Pontos com $r \\leq 2h$')],
              loc='upper left', fontsize=8)
    ax.set_title(r'Núcleo spline cúbico tridimensional: valores no interior do suporte ($h=1,0$)')
    return save(fig, outdir, 'teoria_04_nucleo_3d_nuvem', pdf)


def _quartic_2d(kt):
    Potential, _ = _potential_classes()
    T = TEORIA
    return Potential(xc=T['xc'], yc=T['yc'], R=T['R'], shape='quartic', gain=T['k'] / 2.0, kt=kt)


def _quartic_grid(p, span=1.25, n=241):
    xs = np.linspace(p.xc - span, p.xc + span, n)
    ys = np.linspace(p.yc - span, p.yc + span, n)
    X, Y = np.meshgrid(xs, ys)
    return xs, ys, X, Y


def teoria_campo_quartico(outdir, kt=0.0, pdf=False):
    p = _quartic_2d(kt)
    xs, ys, X, Y = _quartic_grid(p, n=81)
    U = np.zeros_like(X)
    Vv = np.zeros_like(X)
    for i in range(X.shape[0]):
        for j in range(X.shape[1]):
            U[i, j], Vv[i, j] = p.force(X[i, j], Y[i, j])
    M = np.hypot(U, Vv)
    fig, ax = new_fig(7.5, 6.2)
    st = ax.streamplot(xs, ys, U, Vv, color=M, cmap='viridis', density=1.4, linewidth=1.0, arrowsize=1.0)
    _, _, Xf, Yf = _quartic_grid(p)
    ax.contour(Xf, Yf, p.alpha1(Xf, Yf), levels=[0.0], colors='k', linewidths=2.0)
    ax.plot(p.xc, p.yc, 'k+', ms=10, mew=1.5)
    cb = fig.colorbar(st.lines, ax=ax)
    cb.set_label(r'Módulo da força potencial, $\|\mathbf{f}\|$ (u.a.)')    
    cb.formatter = _VirgulaDecimal()
    cb.update_ticks()
    ax.legend(handles=[Line2D([], [], color='k', lw=2.0, label=r'Curva-alvo ($\alpha_1 = 0$)'),
                       Line2D([], [], ls='', marker='+', color='k', ms=10, mew=1.5, label='Centro do potencial')],
              loc='upper right', fontsize=8)
    ax.set_aspect('equal')
    ax.set_xlim(xs[0], xs[-1])
    ax.set_ylim(ys[0], ys[-1])
    ax.set_xlabel('Posição x (m)')
    ax.set_ylabel('Posição y (m)')
    if kt:
        # ax.set_title(f'Campo vetorial do potencial quártico com termo tangencial')
        name = 'teoria_06_campo_quartico_tangencial'
    else:
        # ax.set_title(f'Campo vetorial do potencial quártico')
        name = 'teoria_05_campo_quartico'
    return save(fig, outdir, name, pdf)


def teoria_nivel_quartico(outdir, pdf=False):
    from matplotlib.colors import TwoSlopeNorm
    p = _quartic_2d(0.0)
    xs, ys, X, Y = _quartic_grid(p)
    A = p.alpha1(X, Y) / p.R
    Ac = np.clip(A, -1.0, 4.0)
    fig, ax = new_fig(7.5, 6.2)
    cf = ax.contourf(X, Y, Ac, levels=np.linspace(-1.0, 4.0, 21), cmap='RdBu_r', norm=TwoSlopeNorm(0.0, -1.0, 4.0))
    cs = ax.contour(X, Y, A, levels=[-0.75, -0.5, -0.25, 0.5, 1, 2, 3], colors='k', linewidths=0.5, alpha=0.6)
    ax.clabel(cs, fmt=lambda v: f'{v:g}'.replace('.', ','), fontsize=7)
    ax.contour(X, Y, A, levels=[0.0], colors='k', linewidths=2.0)
    ax.plot(p.xc, p.yc, 'k+', ms=10, mew=1.5)
    cb = fig.colorbar(cf, ax=ax)
    cb.set_label(r'$\alpha_1/R$ (saturado em 4)')
    cb.formatter = _VirgulaDecimal()
    cb.update_ticks()
    ax.legend(handles=[Line2D([], [], color='k', lw=2.0, label=r'Curva-alvo ($\alpha_1 = 0$)'),
                       Line2D([], [], color='k', lw=0.5, label=r'Curvas de nível de $\alpha_1/R$')],
              loc='upper right', fontsize=8)
    ax.set_aspect('equal')
    ax.grid(False)
    ax.set_xlabel('Posição x (m)')
    ax.set_ylabel('Posição y (m)')
    ax.set_title(rf'Função de nível $\alpha_1$ do potencial quártico ($a = {p.a:g}$, $b = {p.b:g}$, '
                 rf'$c = {p.c:g}$, R = {num(p.R, 2)})')
    return save(fig, outdir, 'teoria_07_nivel_quartico', pdf)


def teoria_superficies_3d(outdir, pdf=False):
    from matplotlib.patches import Patch
    _, Potential3D = _potential_classes()
    T = TEORIA
    p = Potential3D(xc=T['xc'], yc=T['yc'], zc=0.0, R=T['R'], shape='quartic')
    q = dict(xc=p.xc, yc=p.yc, R=p.R, a=p.a, b=p.b, c=p.c)
    cx, cy = quartic_curve(q, 300)
    cz = (cx + cy) / 2.0                                   # alpha2 = 0  <=>  z = (x + y) / 2
    z_lo, z_hi = float(cz.min()) - 0.35, float(cz.max()) + 0.35
    zz = np.linspace(z_lo, z_hi, 2)
    CX, CZ = np.meshgrid(cx, zz)
    CY, _ = np.meshgrid(cy, zz)
    span = 1.1 * max(float(np.abs(cx - p.xc).max()), float(np.abs(cy - p.yc).max()))
    g = np.linspace(-span, span, 2)
    PX, PY = np.meshgrid(p.xc + g, p.yc + g)
    PZ = (PX + PY) / 2.0
    fig, ax = new_fig(8.0, 7.0, projection='3d')
    ax.plot_surface(CX, CY, CZ, color='tab:blue', alpha=0.25, lw=0, shade=False)
    ax.plot_surface(PX, PY, PZ, color='tab:orange', alpha=0.30, lw=0, shade=False)
    ax.plot(cx, cy, cz, 'k', lw=2.4)
    ax.plot(cx, cy, np.full_like(cx, z_lo), color='tab:blue', ls='--', lw=1.0)
    ax.set_box_aspect((1, 1, (z_hi - z_lo) / (2 * span)))
    ax.set_xlabel('Posição x (m)')
    ax.set_ylabel('Posição y (m)')
    ax.set_zlabel('Altitude z (m)', labelpad=12)
    ax.legend(handles=[Patch(color='tab:blue', alpha=0.4, label=r'Superfície $\alpha_1 = 0$ (cilindro de seção quártica)'),
                       Patch(color='tab:orange', alpha=0.4, label=r'Superfície $\alpha_2 = 0$ (plano $z = (x + y)/2$)'),
                       Line2D([], [], color='k', lw=2.4, label=r'Curva-alvo ($\alpha_1 = \alpha_2 = 0$)'),
                       Line2D([], [], color='tab:blue', ls='--', lw=1.0, label='Seção quártica (projeção)')],
              loc='upper left', bbox_to_anchor=(1.02, 1.0), borderaxespad=0.0)
    ax.set_title(rf'Curva-alvo tridimensional como interseção de duas superfícies ($b = {p.b:g}$, R = {num(p.R, 2)})')
    return save(fig, outdir, 'teoria_08_superficies_quartico_3d', pdf)


def teoria_campo_circular(outdir, pdf=False):
    """Circle potential with beta = 1: the controller's own Potential(shape='circle').force()."""
    Potential, _ = _potential_classes()
    T = TEORIA
    R = T['R_circ']
    p = Potential(xc=0.0, yc=0.0, R=R, shape='circle')
    g = np.linspace(-1.8 * R, 1.8 * R, 19)
    X, Y = np.meshgrid(g, g)
    U = np.zeros_like(X)
    Vv = np.zeros_like(X)
    for i in range(X.shape[0]):
        for j in range(X.shape[1]):
            U[i, j], Vv[i, j] = p.force(X[i, j], Y[i, j], 1.0, T['k_circ'])
    inside = np.hypot(X, Y) < R
    fig, ax = new_fig(7.0, 6.2)
    ax.quiver(X[~inside], Y[~inside], U[~inside], Vv[~inside], color='tab:blue', angles='xy', pivot='mid',
              width=0.004, scale=T['k_circ'] * 30, label='Força no exterior (atrativa)')
    ax.quiver(X[inside], Y[inside], U[inside], Vv[inside], color='tab:orange', angles='xy', pivot='mid',
              width=0.004, scale=T['k_circ'] * 30, label='Força no interior (repulsiva)')
    th = np.linspace(0, 2 * math.pi, 300)
    ax.plot(R * np.cos(th), R * np.sin(th), 'k', lw=2.0, label=f'Circunferência-alvo ($R$ = {num(R, 0)})')
    ax.plot(0, 0, 'k+', ms=10, mew=1.5, label='Centro')
    ax.set_aspect('equal')
    ax.grid(False)
    ax.set_xlabel(r'$x$')
    ax.set_ylabel(r'$y$')
    legend_out(ax)
    # ax.set_title(r'Campo de forças do potencial circular ($\beta = 1$)')
    return save(fig, outdir, 'teoria_09_campo_circular', pdf)


def teoria_perfil_circular(outdir, pdf=False):
    """Radial component of the circle force along a ray from the centre, normalised by k, beta = 1 and 0."""
    R = TEORIA['R_circ']
    r = np.linspace(0.0, 2.0 * R, 801)
    s = r ** 2 / R ** 2 - 1.0
    f1 = -np.sign(s)
    f1[np.abs(s) < 1e-12] = np.nan
    f0 = -4.0 / R ** 2 * s * r
    fig, ax = new_fig()
    ax.plot(r, f1, 'tab:blue', lw=1.8, label=r'$\beta = 1$ (intensidade constante, descontínua em $r = R$)')
    ax.plot(r, f0, 'tab:red', ls='--', lw=1.6, label=r'$\beta = 0$ (contínua, crescente com a distância)')
    ax.axvline(R, color='k', ls=':', lw=1.0, label=r'Circunferência-alvo ($r = R$)')
    ax.axhline(0, color='k', lw=0.6)
    ax.set_xlim(0, 2.0 * R)
    ax.set_ylim(-3.0, 3.0)
    ax.set_xlabel(r'Distância ao centro, $r/R$')
    ax.set_ylabel(r'Componente radial da força, $f_r/k$')
    ax.set_title('Componente radial da força do potencial circular')
    legend_out(ax)
    return save(fig, outdir, 'teoria_10_perfil_circular', pdf)

def teoria_campo_esferico(outdir, pdf=False):
    """Sphere potential with beta = 1: the controller's own Potential3D(shape='sphere').force(), sampled on
    one shell inside (0.5 R) and one outside (1.6 R) the target sphere."""
    _, Potential3D = _potential_classes()
    T = TEORIA
    R = T['R_circ']
    p = Potential3D(xc=0.0, yc=0.0, zc=0.0, R=R, shape='sphere')
    fig, ax = new_fig(8.0, 7.0, projection='3d')
    u, v = np.meshgrid(np.linspace(0, 2 * math.pi, 40), np.linspace(0, math.pi, 20))
    ax.plot_wireframe(R * np.cos(u) * np.sin(v), R * np.sin(u) * np.sin(v), R * np.cos(v),
                      color='0.55', lw=0.4, alpha=0.6)
    L = 0.20 * R                                              # drawn arrow length (all forces have size k)
    for rad, n, col in ((1.6 * R, 60, 'tab:blue'), (0.5 * R, 14, 'tab:orange')):
        k_ = np.arange(n) + 0.5                               # evenly spread points (golden-angle spiral)
        zz = 1.0 - 2.0 * k_ / n
        rr = np.sqrt(1.0 - zz ** 2)
        ga = math.pi * (3.0 - math.sqrt(5.0)) * k_
        P = rad * np.column_stack([rr * np.cos(ga), rr * np.sin(ga), zz])
        F = np.array([p.force(x, y, z, 1.0, T['k_circ']) for x, y, z in P], dtype=float)
        F = F / np.linalg.norm(F, axis=1, keepdims=True) * L
        ax.quiver(P[:, 0], P[:, 1], P[:, 2], F[:, 0], F[:, 1], F[:, 2], color=col, lw=1.2,
                  arrow_length_ratio=0.35, normalize=False)
    lim = 1.9 * R
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_zlim(-lim, lim)
    for setter in (ax.set_xticks, ax.set_yticks, ax.set_zticks):
        setter([-1, 0, 1])
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=20, azim=-60)
    ax.set_xlabel(r'$x$')
    ax.set_ylabel(r'$y$')
    ax.set_zlabel(r'$z$')
    ax.legend(handles=[Line2D([], [], color='0.55', lw=1, label=f'Esfera-alvo ($R$ = {num(R, 0)})'),
                       Line2D([], [], color='tab:blue', lw=1.5, label='Força no exterior (atrativa)'),
                       Line2D([], [], color='tab:orange', lw=1.5, label='Força no interior (repulsiva)')],
              loc='upper left', bbox_to_anchor=(1.02, 1.0), borderaxespad=0.0)
    ax.set_title(r'Campo de forças do potencial esférico ($\beta = 1$)')
    return save(fig, outdir, 'teoria_11_campo_esferico', pdf)

def teoria_curva_generica(outdir, pdf=False):
    """Generic illustration of eq. (curva_implicita) in R^3: a vertical cylinder with a three-lobed section
    (alpha_1 = 0) cut by a curved surface z = 0.6 (x^2 - y^2) + 0.3 x (alpha_2 = 0); the closed curve is
    their intersection."""
    th = np.linspace(0, 2 * math.pi, 361)
    r = 1.0 + 0.3 * np.cos(3 * th)                           # alpha_1 = 0: three-lobed closed section
    cx, cy = r * np.cos(th), r * np.sin(th)
    surf_z = lambda x, y: 0.6 * (x ** 2 - y ** 2) + 0.3 * x   # alpha_2 = 0
    cz = surf_z(cx, cy)
    fig, ax = new_fig(8.0, 6.8, projection='3d')
    g = np.linspace(-1.6, 1.6, 33)
    SX, SY = np.meshgrid(g, g)
    SZ = surf_z(SX, SY)
    ax.plot_surface(SX, SY, SZ, color='mediumorchid', alpha=0.40, lw=0.3, edgecolor='0.35', rstride=1, cstride=1)
    zz = np.linspace(-2.2, 2.2, 23)
    CX, CZ = np.meshgrid(cx, zz)
    CY, _ = np.meshgrid(cy, zz)
    ax.plot_surface(CX, CY, CZ, color='deepskyblue', alpha=0.35, lw=0.3, edgecolor='0.35', rstride=1, cstride=12)
    ax.plot(cx, cy, cz, 'k', lw=3.0, zorder=10)
    ax.set_xlabel(r'$x_1$')
    ax.set_ylabel(r'$x_2$')
    ax.set_zlabel(r'$x_3$')
    ax.set_box_aspect((1, 1, 0.9))
    ax.view_init(elev=25, azim=-55)
    from matplotlib.patches import Patch
    ax.legend(handles=[Patch(color='deepskyblue', alpha=0.5, label=r'Superfície $\alpha_1 = 0$ (cilindro)'),
                       Patch(color='mediumorchid', alpha=0.5, label=r'Superfície $\alpha_2 = 0$'),
                       Line2D([], [], color='k', lw=3.0, label=r'Curva $\Gamma$ ($\alpha_1 = \alpha_2 = 0$)')],
              loc='upper left', bbox_to_anchor=(1.02, 1.0), borderaxespad=0.0)
    ax.set_title(r'Curva em $\mathbb{R}^3$ definida pela interseção de duas superfícies de nível')
    return save(fig, outdir, 'teoria_12_curva_generica', pdf)
def _obs_accel(X, Y, O):
    """SPH pressure acceleration of the virtual particles O (M x 2) on a test agent at rest at each (X, Y)
    (same W, state equation, B = 20 rho / gamma and m_d = lambda rho_0 / sigma as the controller)."""
    T = TEORIA
    h, m, rho0, gam, lam = T['obs_h'], T['obs_m'], T['obs_rho0'], T['obs_gamma'], T['obs_lambda']
    sig = SIGMA[2] / h ** 2
    m_d = lam * rho0 / sig
    W = lambda r: sig * _spline_f(r / h)[0]
    P_of = lambda rho: (20.0 * rho / gam) * ((rho / rho0) ** gam - 1.0)
    Dd = np.hypot(O[:, None, 0] - O[None, :, 0], O[:, None, 1] - O[None, :, 1])
    rho_dd = m_d * W(Dd).sum(axis=1)
    DX = X[..., None] - O[:, 0]
    DY = Y[..., None] - O[:, 1]
    Rr = np.hypot(DX, DY)
    Wid = W(Rr)
    rho_i = m * sig + m_d * Wid.sum(axis=-1)
    rho_d = rho_dd + m * Wid
    dw = _spline_f(Rr / h)[1] * sig / h
    with np.errstate(invalid='ignore', divide='ignore'):
        gx = np.where(Rr > 0, dw * DX / Rr, 0.0)
        gy = np.where(Rr > 0, dw * DY / Rr, 0.0)
    coef = m_d * (P_of(rho_i)[..., None] / rho_i[..., None] ** 2 + P_of(rho_d) / rho_d ** 2)
    return -(coef * gx).sum(axis=-1), -(coef * gy).sum(axis=-1)


def teoria_campo_obstaculos(outdir, pdf=False):
    """Field felt by an agent at rest: quartic attraction (controller's Potential, without circulation) plus
    obstacles made of a single virtual particle each (TEORIA['obs_pts'])."""
    T = TEORIA
    p = _quartic_2d(0.0)
    x0, x1, y0, y1 = T['obs_view']
    xs, ys = np.linspace(x0, x1, 161), np.linspace(y0, y1, 161)
    X, Y = np.meshgrid(xs, ys)
    U = np.zeros_like(X)
    Vv = np.zeros_like(X)
    for i in range(X.shape[0]):
        for j in range(X.shape[1]):
            U[i, j], Vv[i, j] = p.force(X[i, j], Y[i, j])
    O = np.array(T['obs_pts'], dtype=float)
    OX, OY = _obs_accel(X, Y, O)
    cap = 2.0 * float(np.nanmax(np.hypot(U, Vv)))             # colour saturates at 2x the potential's maximum
    U, Vv = U + OX, Vv + OY
    M = np.minimum(np.hypot(U, Vv), cap)
    fig, ax = new_fig(7.5, 6.8)
    st = ax.streamplot(xs, ys, U, Vv, color=M, cmap='viridis', norm=plt.Normalize(0.0, cap),
                       density=1.6, linewidth=0.9, arrowsize=1.1)
    Xf, Yf = np.meshgrid(np.linspace(x0, x1, 481), np.linspace(y0, y1, 481))
    ax.contour(Xf, Yf, p.alpha1(Xf, Yf), levels=[0.0], colors='k', linewidths=2.0)
    for ox, oy in O:
        ax.add_patch(plt.Circle((ox, oy), T['obs_radius'], facecolor='0.75', edgecolor='k', lw=1.2, zorder=4))
    ax.plot(O[:, 0], O[:, 1], 'o', color='tab:red', mec='k', ms=6, zorder=5)
    cb = fig.colorbar(st.lines, ax=ax)
    cb.set_label(rf'Módulo da aceleração (m/s$^2$)')
    cb.formatter = _VirgulaDecimal()
    cb.update_ticks()
    ax.legend(handles=[Line2D([], [], color='k', lw=2.0, label=r'Curva-alvo ($\alpha_1 = 0$)'),
                       Line2D([], [], ls='', marker='o', color='0.75', mec='k', ms=12, label=rf'Obstáculo (raio {num(T["obs_radius"], 1)} m)'),
                       Line2D([], [], ls='', marker='o', color='tab:red', mec='k', ms=6, label='Partícula virtual')],
              loc='upper center', bbox_to_anchor=(0.5, -0.11), ncol=3, fontsize=8, frameon=False)
    ax.set_aspect('equal')
    ax.set_xlim(xs[0], xs[-1])
    ax.set_ylim(ys[0], ys[-1])
    ax.set_xlabel('Posição x (m)')
    ax.set_ylabel('Posição y (m)')
    return save(fig, outdir, 'teoria_13_campo_obstaculos', pdf)

def make_teoria(outdir, pdf=False):
    os.makedirs(outdir, exist_ok=True)
    made = [teoria_nucleo_perfil(outdir, 2, pdf), teoria_nucleo_2d_superficie(outdir, pdf),
            teoria_nucleo_perfil(outdir, 3, pdf), teoria_nucleo_3d_nuvem(outdir, pdf),
            teoria_campo_quartico(outdir, 0.0, pdf)]
    if TEORIA['kt']:
        made.append(teoria_campo_quartico(outdir, TEORIA['kt'], pdf))
    made += [teoria_nivel_quartico(outdir,   pdf), teoria_superficies_3d(outdir, pdf)]
    made += [teoria_campo_circular(outdir, pdf), teoria_perfil_circular(outdir, pdf)]
    made.append(teoria_campo_esferico(outdir, pdf))
    made.append(teoria_curva_generica(outdir, pdf))
    made.append(teoria_campo_obstaculos(outdir, pdf))
    return [m for m in made if m]


OLD_PNGS = ['00_summary.png', '01_trajectories.png', '02_separation.png', '03_sph_dynamics.png',
            '04_performance.png', '05_3d.png']


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('run', nargs='?', default='latest', help='run folder, logs folder, or `latest` (default)')
    ap.add_argument('--out', default=None, help='output folder (default <run>/plots)')
    ap.add_argument('--collide', type=float, default=0.10, help='centre-to-centre distance counted as a collision (m)')
    ap.add_argument('--target-spacing', type=float, default=0.3, help='wanted neighbour spacing for the formation figure (m)')
    ap.add_argument('--no-open', action='store_true', help='only write the PNGs: do not open windows / do not wait')
    ap.add_argument('--open', action='store_true', help=argparse.SUPPRESS)   # old flag, now the default
    ap.add_argument('--pdf', action='store_true', help='also write a vector PDF of every figure (for LaTeX)')
    ap.add_argument('--teoria', action='store_true',
                    help='only the methodology figures (kernel 2D/3D, quartic field and surfaces; see TEORIA), '
                         'no run needed; written to src/plots/teoria (or --out)')
    a = ap.parse_args()
    if a.teoria:
        out = a.out or os.path.join(HERE, 'plots', 'teoria')
        made = make_teoria(out, a.pdf)
        for m in made:
            print(f'  saved {m}')
        if made and not a.no_open:
            show_and_wait(made)
        return
    path = a.run
    if path == 'latest':
        runs = os.path.join(HERE, 'logs', 'runs')
        lat = os.path.join(runs, 'latest')
        path = lat if os.path.isdir(lat) else ar.newest_run(os.path.join(HERE, 'logs'))
    D = RunData(path, a.collide)
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
    for old in OLD_PNGS:                  # figures of the previous layout (several charts per image)
        if os.path.exists(os.path.join(out, old)):
            os.remove(os.path.join(out, old))
    pdf = a.pdf
    made = [fig_resumo(D, res, out, pdf), fig_trajetorias_xy(D, out, pdf), fig_trajetorias_3d(D, out, pdf),
            fig_altitude(D, out, pdf), fig_erro_alvo(D, out, pdf)]
    made += fig_formacao(D, out, a.target_spacing, pdf=pdf)
    made += [fig_distancia_minima(D, res, out, pdf), fig_vizinho(D, res, out, pdf),
             fig_balanco_densidade(D, res, out, pdf)]
    made += figs_sph(D, out, pdf) + figs_desempenho(D, out, pdf)
    made = [m for m in made if m]
    print(f'run {D.run.name}: {D.n} drones, window {res["window"][0]:.1f}..{res["window"][1]:.1f} s')
    if 'dmin' in res:
        print(f'  closest approach {res["dmin"]:.3f} m | collisions (<{a.collide:g} m): {len(res["ep_collide"])} '
              f'| median NN {res["nn_med"]:.3f} m')
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