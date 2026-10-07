#!/usr/bin/env python3
"""Turn one run folder into a report you (and Claude) can read in two minutes.

    python3 analyze_run.py logs/runs/latest            # newest run
    python3 analyze_run.py logs/runs/<run_id>
    python3 analyze_run.py logs                        # newest run under logs/runs
    python3 analyze_run.py A B C --compare             # side-by-side table of several runs
    python3 analyze_run.py logs/plain_sph_cf_*.csv     # OLD 9-column logs (limited analysis)

Writes <run>/analysis/:
    summary.txt     ~40 lines: verdicts + the key numbers.  PASTE THIS to Claude first.
    report.md       the full report: health checks, per-drone tables, diagnosis, glossary
    report.json     every metric as numbers (used by --compare)
    fig_*.png       overview, tracking, spectrum, forces, trajectory, timing

Needs only numpy + matplotlib.  What every check means and what to do about a
WARN/FAIL is in TEST_GUIDE.md (the check ids C1..C14 are the same).
"""
import argparse
import csv
import glob
import json
import math
import os
import re
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ----------------------------------------------------------------------------- thresholds
TH = {
    'rx_rate_min_frac': 0.85,     # state rate must be >= 85 % of nominal (in wall time)
    'rtf_warn': 0.85, 'rtf_fail': 0.5,
    'stale_warn': 10.0, 'stale_fail': 30.0,       # % of ticks without a new state message
    'late_warn': 2.0, 'late_fail': 10.0,          # % of ticks > 1.5 x period late
    'step_ms_warn_frac': 0.3,                     # control step cost / period
    'pred_err_warn': 0.02, 'pred_err_fail': 0.05, # m, p95 (predicted vs measured position)
    'gain_lo': 0.8, 'gain_hi': 1.2, 'r2_warn': 0.8, 'r2_fail': 0.5,
    'lag_warn': 0.3, 'lag_fail': 0.6,             # s (SIM time) delay + time constant
    'osc_prom': 6.0, 'osc_rms': 0.02,             # spectral peak / median ; m/s rms
    'grow_slope': 0.03,                           # 1/s on ln(rms)
    'drift_warn': 0.01, 'drift_fail': 0.03,       # m/s measured while commanded ~0
    'clip_warn': 2.0, 'clip_fail': 10.0,          # % ticks saturated
    'chatter_warn': 25.0, 'chatter_fail': 40.0,   # % ticks where cmd flips sign
    'rho_band': 0.10, 'rho_ok_warn': 80.0,        # % of time rho within +-10 % of rho0
    'z_warn': 0.05, 'z_fail': 0.15,               # m from target altitude
}
STATE_HZ_NOMINAL = 20.0


# ----------------------------------------------------------------------------- loading
def read_csv_cols(path):
    with open(path, newline='') as f:
        rd = csv.reader(f)
        header = next(rd, None)
        if header is None:
            return {}
        cols = {h: [] for h in header}
        for row in rd:
            if len(row) != len(header):
                continue                       # truncated last line after a hard kill
            for h, v in zip(header, row):
                cols[h].append(v)
    out = {}
    for h, vals in cols.items():
        if h in ('drone', 'src'):
            out[h] = np.array(vals, dtype=object)
        else:
            out[h] = np.array([float(v) if v not in ('', 'nan') else np.nan for v in vals])
    return out


class Run:
    """Everything known about one run."""

    def __init__(self, path):
        self.legacy = False
        self.meta = {}
        self.ticks, self.rxs = {}, {}
        self.rtf = None
        self.events = []
        if os.path.isdir(path) and os.path.exists(os.path.join(path, 'runs')) and \
                not glob.glob(os.path.join(path, 'tick_*.csv')):
            path = newest_run(path)
        if os.path.isdir(path) and glob.glob(os.path.join(path, 'tick_*.csv')):
            self.dir = os.path.realpath(path)
            self._load_new()
        else:
            self._load_legacy(path)
        self.names = sorted(self.ticks, key=lambda s: (len(s), s))
        self.name = os.path.basename(self.dir.rstrip('/'))

    def _load_new(self):
        d = self.dir
        mp = os.path.join(d, 'meta.json')
        if os.path.exists(mp):
            with open(mp) as f:
                self.meta = json.load(f)
        for p in sorted(glob.glob(os.path.join(d, 'tick_*.csv'))):
            n = os.path.basename(p)[5:-4]
            self.ticks[n] = read_csv_cols(p)
        for p in sorted(glob.glob(os.path.join(d, 'rx_*.csv'))):
            n = os.path.basename(p)[3:-4]
            self.rxs[n] = read_csv_cols(p)
        rp = os.path.join(d, 'rtf.csv')
        if os.path.exists(rp) and os.path.getsize(rp) > 40:
            self.rtf = read_csv_cols(rp)
        else:
            self.rtf = self._parse_gz_text(os.path.join(d, 'rtf.txt'))
        ep = os.path.join(d, 'events.log')
        if os.path.exists(ep):
            with open(ep, errors='replace') as f:
                self.events = [ln.rstrip('\n') for ln in f]

    @staticmethod
    def _parse_gz_text(path):
        """rtf.txt = raw `gz topic -e` dump (same format as the old rtf_log.txt)."""
        if not os.path.exists(path):
            return None
        txt = open(path, errors='replace').read()
        rt = [float(x) for x in re.findall(r'real_time_factor:\s*([0-9.eE+-]+)', txt)]
        if not rt:
            return None
        return {'epoch': np.full(len(rt), np.nan), 'rtf': np.array(rt),
                'sim_time': np.full(len(rt), np.nan), 'real_time': np.full(len(rt), np.nan)}

    def _load_legacy(self, path):
        files = sorted(glob.glob(path)) if any(c in path for c in '*?') else [path]
        files = [f for f in files if f.endswith('.csv')]
        if not files:
            raise SystemExit(f'no run folder or plain_sph_*.csv found at {path}')
        self.legacy = True
        self.dir = os.path.dirname(os.path.realpath(files[0]))
        for f in files:
            n = re.sub(r'^plain_sph_|\.csv$', '', os.path.basename(f))
            c = read_csv_cols(f)
            t = c['t']
            nan = np.full(len(t), np.nan)
            self.ticks[n] = {
                't': t, 'epoch': nan.copy(), 'drone': np.array([n] * len(t), dtype=object),
                'src': np.array(['legacy'] * len(t), dtype=object),
                'rho': c['rho'], 'P': c['pressure'],
                'ae_x': c['ext_force_x'], 'ae_y': c['ext_force_y'],
                'cmd_raw_x': c['vel_x'], 'cmd_raw_y': c['vel_y'],
                'cmd_x': c['vel_x'], 'cmd_y': c['vel_y'],
                'used_x': c.get('x', nan.copy()), 'used_y': c.get('y', nan.copy()),
                'meas_x': c.get('x', nan.copy()), 'meas_y': c.get('y', nan.copy()),
            }
        self.meta = {'label': 'legacy', 'n_drones': len(self.ticks)}


def newest_run(logs):
    base = logs if os.path.basename(logs.rstrip('/')) == 'runs' else os.path.join(logs, 'runs')
    cands = [p for p in glob.glob(os.path.join(base, '*')) if os.path.isdir(p)
             and not os.path.islink(p) and glob.glob(os.path.join(p, 'tick_*.csv'))]
    if not cands:
        raise SystemExit(f'no run folders in {base}')
    return max(cands, key=os.path.getmtime)


# ----------------------------------------------------------------------------- helpers
def g(col, default=np.nan):
    return default if col is None else col


def pct(x, q):
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    return float(np.percentile(x, q)) if len(x) else float('nan')


def fnum(x, nd=2, unit=''):
    if x is None or (isinstance(x, float) and not math.isfinite(x)):
        return 'n/a'
    return f'{x:.{nd}f}{unit}'


def sub(col, mask):
    return col[mask] if col is not None else None


def kernel(r, h):
    k = r / h
    c = 10.0 / (7.0 * math.pi * h * h)
    if k <= 1.0:
        return c * (1 - 1.5 * k ** 2 + 0.75 * k ** 3)
    if k <= 2.0:
        return c * 0.25 * (2 - k) ** 3
    return 0.0


def ring_density(n, radius, mass, h):
    return mass * sum(kernel(2 * radius * math.sin(math.pi * j / n), h) for j in range(n))


def equilibrium_ring_radius(n, mass, h, rho0):
    """Radius of an evenly spaced ring whose SPH density equals rho0 (pressure zero)."""
    if n < 2:
        return float('nan')
    lo, hi = 0.01, 100.0
    if ring_density(n, hi, mass, h) > rho0 or ring_density(n, lo, mass, h) < rho0:
        return float('nan')
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if ring_density(n, mid, mass, h) > rho0:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


# ----------------------------------------------------------------------------- analysis
class Analysis:
    def __init__(self, run, skip=2.0):
        self.r = run
        self.skip = skip
        self.m = {}          # flat metrics (report.json)
        self.checks = []     # (id, title, status, value, rule)
        self.notes = []      # diagnosis lines (severity, text)
        self.per = {}        # per-drone table
        self.names = run.names
        meta = run.meta or {}
        core = meta.get('core', {})
        self.params = meta.get('params', {})
        self.period = core.get('period_s', 0.05)
        self.dt_model = core.get('dt_model_s', self.period)
        self.fixed_z = core.get('fixed_z', 0.5)
        self.sph = core.get('sph', {})
        self.n = len(self.names)
        self.max_v = core.get('max_velocity', 0.5)
        self.rtf_eff = 1.0
        self.sim_clock = core.get('sim_clock', 'wall') == 'gz'
        self.real = str(self.params.get('platform', 'sim')) == 'real'
        self._phases()
        self._rtf()
        if self.rtf_mean is not None:
            self.rtf_eff = self.rtf_mean

    # ---- time axes ---------------------------------------------------------
    def _phases(self):
        r = self.r
        t_all = np.concatenate([r.ticks[n]['t'] for n in self.names])
        self.t0, self.t1 = float(np.nanmin(t_all)), float(np.nanmax(t_all))
        thr = 0.8 * self.fixed_z
        starts = []
        for n in self.names:
            T = r.ticks[n]
            z = T.get('meas_z')
            if z is None or not np.isfinite(z).any():
                starts = []
                break
            t = T['t']
            ok = z >= thr
            st = None
            for i in range(len(t)):
                if ok[i]:
                    if st is None:
                        st = t[i]
                    if t[i] - st >= 1.0:
                        break
                else:
                    st = None
            else:
                st = None
            if st is None or (len(t) and not ok[-1] and t[-1] - st < 1.0):
                starts = []
                break
            starts.append(st)
        self.flight_start = max(starts) if starts else None
        self.w0 = (self.flight_start if self.flight_start is not None else self.t0) + self.skip
        self.w1 = self.t1
        # divergence: a Crazyflie commanded <= 0.5 m/s never really moves at > 3 m/s or climbs 1 m above
        # the target; if it does, the estimate blew up / the drone crashed and every later number is garbage
        self.diverged_at, self.diverged_why = None, ''
        for n in self.names:
            T = r.ticks[n]
            if 'meas_vx' not in T or not np.isfinite(T['meas_vx']).any():
                continue
            sp = np.hypot(T['meas_vx'], T['meas_vy'])
            bad = (sp > 3.0) | (T['meas_z'] > self.fixed_z + 1.0) | ~np.isfinite(sp)
            idx = np.where(bad)[0]
            if len(idx) and (self.diverged_at is None or T['t'][idx[0]] < self.diverged_at):
                self.diverged_at = float(T['t'][idx[0]])
                i = idx[0]
                self.diverged_why = (f'{n}: |v| = {sp[i]:.1f} m/s, z = {T["meas_z"][i]:.2f} m')
        if self.diverged_at is not None:
            self.w1 = min(self.w1, self.diverged_at - 0.5)
        self.window_ok = self.w1 - self.w0 >= 3.0
        # first row of a test profile
        self.test_t0 = None
        for n in self.names:
            s = r.ticks[n].get('src')
            if s is not None:
                idx = [i for i, v in enumerate(s) if str(v).startswith('test:')]
                if idx:
                    t = r.ticks[n]['t'][idx[0]]
                    self.test_t0 = t if self.test_t0 is None else min(self.test_t0, t)
        self.test_kind = None
        for n in self.names:
            s = r.ticks[n].get('src')
            if s is not None:
                u = {str(v) for v in s if str(v).startswith('test:')}
                if u:
                    self.test_kind = sorted(u)[0][5:]
                    break
        if self.test_t0 is not None:
            self.w0 = max(self.w0, self.test_t0)

    def mask(self, t):
        return (t >= self.w0) & (t <= self.w1)

    def _rtf(self):
        """Mean real-time factor in the window (rtf.csv) or None."""
        self.rtf_mean, self.rtf_series, self.rtf_src = None, None, 'unknown'
        R = self.r.rtf
        if R is None:
            return
        rtf = np.asarray(R['rtf'], dtype=float)
        ep = np.asarray(R['epoch'], dtype=float)
        if np.isfinite(ep).any():
            # map run time -> epoch with the tick logs
            offs = []
            for n in self.names:
                T = self.r.ticks[n]
                if 'epoch' in T and np.isfinite(T['epoch']).any():
                    offs.append(np.nanmedian(T['epoch'] - T['t']))
            if offs:
                off = float(np.median(offs))
                tt = ep - off
                self.rtf_series = (tt, rtf)
                m = (tt >= self.w0) & (tt <= self.w1) & np.isfinite(rtf)
                if m.any():
                    self.rtf_mean = float(np.mean(rtf[m]))
                    self.rtf_src = 'rtf.csv'
                    return
        # no usable timestamps (raw gz dump): whole-file mean
        if np.isfinite(rtf).any():
            self.rtf_mean = float(np.nanmean(rtf))
            self.rtf_src = 'rtf.txt (whole run, not time-aligned)'

    def add(self, cid, title, status, value, rule):
        self.checks.append((cid, title, status, value, rule))

    # ---- C1/C2/C3: timing & freshness ---------------------------------------
    def timing(self):
        r, W = self.r, self.window_ok
        if r.legacy:
            for cid, t in (('C1', 'state rate'), ('C2', 'tick timing'), ('C3', 'stale ticks'),
                           ('C4', 'prediction error')):
                self.add(cid, t, 'N/A', 'not logged in the old format',
                         'rerun with the new logging to get this')
            return
        rates, jit, gaps = [], [], []
        stale, dbl, late, run_max, pe95, dt50, dt95, dtmax, st95 = [], [], [], [], [], [], [], [], []
        for n in self.names:
            T = r.ticks[n]
            mk = self.mask(T['t'])
            RX = r.rxs.get(n)
            if RX and len(RX.get('t', [])):
                mr = self.mask(RX['t'])
                dts = RX['rx_dt_ms'][mr]
                dts = dts[np.isfinite(dts)]
                if len(dts):
                    rates.append(1000.0 / np.mean(dts))
                    jit.append(float(np.std(dts)))
                    gaps.append(float(np.max(dts)))
            rs = T['rx_since'][mk]
            if len(rs):
                stale.append(100.0 * np.mean(rs == 0))
                dbl.append(100.0 * np.mean(rs >= 2))
            # sim-clock runs tick on SIMULATED time: judge the simulated interval, not the wall one
            wd = (T['sim_dt_ms'] if self.sim_clock and 'sim_dt_ms' in T else T['wall_dt_ms'])[mk]
            wd = wd[np.isfinite(wd)]
            if len(wd):
                late.append(100.0 * np.mean(wd > 1.5 * self.period * 1000))
                dt50.append(pct(wd, 50)); dt95.append(pct(wd, 95)); dtmax.append(float(np.max(wd)))
            sr = T['stale_run'][mk]
            if len(sr):
                run_max.append(float(np.max(sr)))
            pe = T['pred_err'][mk]
            if len(pe):
                pe95.append(pct(pe, 95))
            sm = T['step_ms'][mk]
            if len(sm):
                st95.append(pct(sm, 95))
            self.per.setdefault(n, {}).update({
                'rx_hz': rates[-1] if RX and rates else float('nan'),
                'stale_pct': stale[-1] if stale else float('nan'),
                'dbl_pct': dbl[-1] if dbl else float('nan'),
                'stale_run_max': run_max[-1] if run_max else float('nan'),
                'pred_err_p95': pe95[-1] if pe95 else float('nan'),
            })
        rate = float(np.mean(rates)) if rates else float('nan')
        self.m.update({'rx_rate_hz': rate, 'rx_jitter_ms': float(np.mean(jit)) if jit else float('nan'),
                       'rx_gap_max_ms': float(np.max(gaps)) if gaps else float('nan'),
                       'stale_pct': float(np.mean(stale)) if stale else float('nan'),
                       'double_pct': float(np.mean(dbl)) if dbl else float('nan'),
                       'stale_run_max': float(np.max(run_max)) if run_max else float('nan'),
                       'late_pct': float(np.mean(late)) if late else float('nan'),
                       'tick_dt_p50_ms': float(np.mean(dt50)) if dt50 else float('nan'),
                       'tick_dt_p95_ms': float(np.mean(dt95)) if dt95 else float('nan'),
                       'tick_dt_max_ms': float(np.max(dtmax)) if dtmax else float('nan'),
                       'step_ms_p95': float(np.max(st95)) if st95 else float('nan'),
                       'pred_err_p95_m': float(np.max(pe95)) if pe95 else float('nan')})
        # RTF
        rtf = self.rtf_mean
        if rtf is None and math.isfinite(rate):
            rtf_est = rate / STATE_HZ_NOMINAL
            self.m['rtf_estimated_from_rx'] = rtf_est
        self.m['rtf_mean'] = rtf if rtf is not None else float('nan')
        if rtf is not None:
            lo = pct(self.rtf_series[1][(self.rtf_series[0] >= self.w0) & (self.rtf_series[0] <= self.w1)], 5) \
                if self.rtf_series else float('nan')
            self.m['rtf_p5'] = lo
            st = 'PASS' if rtf >= TH['rtf_warn'] else ('WARN' if rtf >= TH['rtf_fail'] else 'FAIL')
            note = ''
            if self.sim_clock and st == 'FAIL':
                st, note = 'WARN', ' (sim_clock=gz: only slows the run down, not a correctness problem)'
            self.add('C1', 'simulator speed (real-time factor)', st,
                     f'RTF {rtf:.2f} (p5 {fnum(lo)}), source {self.rtf_src}{note}',
                     f'>= {TH["rtf_warn"]} ok; < {TH["rtf_fail"]} fail. The controller ticks in WALL time; '
                     f'the drones live in SIM time.')
        elif self.real and math.isfinite(rate):
            st = 'PASS' if rate >= 17 else ('WARN' if rate >= 12 else 'FAIL')
            self.add('C1', 'state stream rate (real drones: no simulator, time is wall time)', st,
                     f'state arrives at {rate:.1f} Hz (configured 20 Hz); radio bandwidth limits this with many drones',
                     '>= 17 Hz ok; < 12 Hz fail. Lower rates add measurement delay to the SPH loop')
        elif math.isfinite(rate):
            est = rate / STATE_HZ_NOMINAL
            st = 'PASS' if est >= TH['rtf_warn'] else ('WARN' if est >= TH['rtf_fail'] else 'FAIL')
            self.add('C1', 'simulator speed (estimated from state rate)', st,
                     f'state arrives at {rate:.1f} Hz of wall time vs nominal {STATE_HZ_NOMINAL:.0f} Hz '
                     f'=> RTF about {est:.2f} (no rtf.csv in this run)',
                     'if the 20 Hz state topic is paced by simulated time, rate/20 = RTF')
        else:
            self.add('C1', 'simulator speed', 'N/A', 'no rtf.csv and no rx logs', '')
        self.rtf_eff = rtf if rtf is not None else (1.0 if self.real else (rate / STATE_HZ_NOMINAL if math.isfinite(rate) else 1.0))
        self.m['rtf_used'] = self.rtf_eff
        self.m['sim_s_per_tick_ms'] = self.period * 1000 if self.sim_clock else self.rtf_eff * self.period * 1000

        # C2 tick timing
        late_p = self.m['late_pct']
        stp = self.m['step_ms_p95']
        st = 'PASS'
        if late_p > TH['late_fail']:
            st = 'FAIL'
        elif late_p > TH['late_warn'] or stp > TH['step_ms_warn_frac'] * self.period * 1000:
            st = 'WARN'
        self.add('C2', 'control tick timing', st,
                 f'dt p50/p95/max = {fnum(self.m["tick_dt_p50_ms"], 0)}/{fnum(self.m["tick_dt_p95_ms"], 0)}/'
                 f'{fnum(self.m["tick_dt_max_ms"], 0)} ms (nominal {self.period * 1000:.0f}); '
                 f'late {fnum(late_p, 1)} %; step cost p95 {fnum(stp, 2)} ms',
                 f'late > {TH["late_warn"]} % warn, > {TH["late_fail"]} % fail; step cost should stay '
                 f'< {TH["step_ms_warn_frac"] * 100:.0f} % of the period')
        # C3 stale
        sp = self.m['stale_pct']
        st = 'PASS' if sp < TH['stale_warn'] else ('WARN' if sp < TH['stale_fail'] else 'FAIL')
        pol = (self.r.meta.get('core') or {}).get('stale_policy', 'predict')
        self.add('C3', 'ticks without a fresh state message', st,
                 f'{fnum(sp, 1)} % stale (longest run {fnum(self.m["stale_run_max"], 0)} ticks), '
                 f'{fnum(self.m["double_pct"], 1)} % ticks got 2+ messages; stale_policy={pol}',
                 f'< {TH["stale_warn"]} % ok. With policy "predict" a stale tick integrates the '
                 f'controller\'s own prediction again')
        # C4 prediction error
        pe = self.m['pred_err_p95_m']
        st = 'PASS' if pe < TH['pred_err_warn'] else ('WARN' if pe < TH['pred_err_fail'] else 'FAIL')
        self.add('C4', 'predicted vs measured position', st,
                 f'p95 gap {fnum(pe * 100, 1)} cm', f'< {TH["pred_err_warn"] * 100:.0f} cm ok')

    # ---- C5: command tracking -------------------------------------------------
    def tracking(self):
        r = self.r
        if r.legacy or not self.window_ok:
            self.add('C5', 'drone follows velocity commands', 'N/A',
                     'needs measured velocity (new logs) and a flight window', '')
            return
        res = {}
        zero_off = {}
        for n in self.names:
            T, RX = r.ticks[n], r.rxs.get(n)
            if not RX or not len(RX.get('t', [])):
                continue
            tc, cx, cy = T['t'], T['cmd_x'], T['cmd_y']
            ok = np.isfinite(cx) & np.isfinite(cy)
            tc, cx, cy = tc[ok], cx[ok], cy[ok]
            mr = self.mask(RX['t'])
            tm, vx, vy = RX['t'][mr], RX['vx'][mr], RX['vy'][mr]
            if len(tm) < 20 or len(tc) < 20:
                continue
            # (a) offset while commanded ~0: the drone's own drift
            mag = np.hypot(cx, cy)
            past = np.array([mag[(tc >= x - 2.0) & (tc <= x)].max() if ((tc >= x - 2.0) & (tc <= x)).any()
                             else 0.0 for x in tm])
            quiet = past < 0.01          # command ~0 for the last 2 s: the drone has settled
            if quiet.sum() >= 10:
                zero_off[n] = (float(np.mean(vx[quiet])), float(np.mean(vy[quiet])), int(quiet.sum()))
            # (b) dynamic fit (only if the command carries some excitation)
            m = (tc >= self.w0 - 2.0) & (tc <= self.w1)
            exc = float(np.sqrt(np.mean(np.concatenate([cx[m], cy[m]]) ** 2))) if m.any() else 0.0
            fit = self._fit(tc, cx, cy, tm, vx, vy) if exc >= 0.03 else None
            res[n] = {'excitation_rms': exc, 'fit': fit}
        if not res:
            self.add('C5', 'drone follows velocity commands', 'N/A', 'no rx data in window', '')
            return
        fits = [v['fit'] for v in res.values() if v['fit']]
        rtf = self.rtf_eff
        self.m['cmd_rms'] = float(np.mean([v['excitation_rms'] for v in res.values()]))
        self.track = res
        self.zero_off = zero_off
        if fits:
            gain = float(np.mean([f['gain'] for f in fits]))
            r2 = float(np.mean([f['r2'] for f in fits]))
            lag = float(np.mean([f['lag'] for f in fits]))
            tau = float(np.mean([f['tau'] for f in fits]))
            tot_sim = (lag + tau) * rtf
            self.m.update({'track_gain': gain, 'track_r2': r2, 'track_delay_wall_s': lag,
                           'track_tau_wall_s': tau, 'track_lag_sim_s': tot_sim})
            bad = (gain < TH['gain_lo'] or gain > TH['gain_hi'] or r2 < TH['r2_warn'])
            fail = (r2 < TH['r2_fail'] or tot_sim > TH['lag_fail'])
            st = 'FAIL' if fail else ('WARN' if bad or tot_sim > TH['lag_warn'] else 'PASS')
            closed = '' if self.test_kind else ' (closed loop: SPH is driving, estimate is biased; run a test profile for a clean one)'
            self.add('C5', 'drone follows velocity commands', st,
                     f'model v = gain * lowpass(cmd delayed): gain {gain:.2f}, delay {lag * 1000:.0f} ms, '
                     f'tau {tau * 1000:.0f} ms (wall) => total lag {tot_sim * 1000:.0f} ms of sim time, '
                     f'R2 {r2:.2f}{closed}',
                     f'gain {TH["gain_lo"]}-{TH["gain_hi"]}, R2 >= {TH["r2_warn"]}, sim-time lag < {TH["lag_warn"]} s')
            for n, v in res.items():
                if v['fit']:
                    self.per.setdefault(n, {}).update({
                        'gain': v['fit']['gain'], 'r2': v['fit']['r2'],
                        'delay_ms': v['fit']['lag'] * 1000, 'tau_ms': v['fit']['tau'] * 1000})
        else:
            self.add('C5', 'drone follows velocity commands', 'INFO',
                     f'command rms only {self.m["cmd_rms"]:.3f} m/s: too small to identify the drone model '
                     f'(this is normal for a calm SPH run). Run test T3 (step/chirp) for gain and lag.',
                     'needs |cmd| rms >= 0.03 m/s')

        # C6: hover drift
        if zero_off:
            sp = {n: math.hypot(a, b) for n, (a, b, _) in zero_off.items()}
            worst = max(sp.values())
            self.m['hover_drift_mps'] = float(worst)
            self.m['hover_drift_mean_mps'] = float(np.mean(list(sp.values())))
            st = 'PASS' if worst < TH['drift_warn'] else ('WARN' if worst < TH['drift_fail'] else 'FAIL')
            txt = ', '.join(f'{n}: {math.hypot(a, b) * 1000:.0f} mm/s' for n, (a, b, _) in zero_off.items())
            self.add('C6', 'drone speed when commanded ~0 (own drift)', st, txt,
                     f'< {TH["drift_warn"] * 1000:.0f} mm/s ok. Non-zero = the drone/estimator moves by itself')
            for n, (a, b, k) in zero_off.items():
                self.per.setdefault(n, {})['hover_vx'], self.per[n]['hover_vy'] = a, b
        else:
            self.add('C6', 'drone speed when commanded ~0 (own drift)', 'INFO',
                     'no stretch with |cmd| < 1 cm/s for 2 s in the window; run test T2 (zero command)', '')

    def mean_motion(self):
        """C15: what each drone is commanded on average vs how it actually moves.
        A persistent command with ~zero measured velocity = the controller is holding
        against something (drone drift, potential, SPH imbalance)."""
        r = self.r
        if r.legacy or not self.window_ok or self.test_kind:
            return
        rows = {}
        for n in self.names:
            T = r.ticks[n]
            m = self.mask(T['t'])
            if m.sum() < 20 or not np.isfinite(T['cmd_x'][m]).any():
                continue
            cx, cy = float(np.nanmean(T['cmd_x'][m])), float(np.nanmean(T['cmd_y'][m]))
            vx, vy = float(np.nanmean(T['meas_vx'][m])), float(np.nanmean(T['meas_vy'][m]))
            ox = float(np.nanmean(T['meas_x'][m] - T['cen_x'][m]))
            oy = float(np.nanmean(T['meas_y'][m] - T['cen_y'][m]))
            nrm = math.hypot(ox, oy) or 1.0
            rows[n] = {'cmd': math.hypot(cx, cy), 'v': math.hypot(vx, vy),
                       'cmd_rad': (cx * ox + cy * oy) / nrm, 'v_rad': (vx * ox + vy * oy) / nrm}
            self.per.setdefault(n, {}).update({'mean_cmd': rows[n]['cmd'], 'mean_v': rows[n]['v'],
                                               'mean_cmd_radial': rows[n]['cmd_rad'],
                                               'mean_v_radial': rows[n]['v_rad']})
        if not rows:
            return
        worst = max(rows.values(), key=lambda d: d['cmd'])
        wn = [n for n, d in rows.items() if d is worst][0]
        self.m['mean_cmd_max_mps'] = worst['cmd']
        self.m['mean_v_max_mps'] = max(d['v'] for d in rows.values())
        holding = worst['cmd'] > 0.01 and worst['v'] < 0.6 * worst['cmd']
        txt = '; '.join(f'{n}: cmd {d["cmd"] * 1000:.0f} mm/s (radial {d["cmd_rad"] * 1000:+.0f}), '
                        f'moves {d["v"] * 1000:.0f} mm/s' for n, d in rows.items())
        self.add('C15', 'average command vs average motion', 'WARN' if holding else 'INFO', txt,
                 'a persistent command with almost no motion = the controller is holding against a '
                 'constant push (drone drift, potential force, or an SPH imbalance)')

    @staticmethod
    def _zoh(tc, c, tq):
        i = np.searchsorted(tc, tq, side='right') - 1
        return c[np.clip(i, 0, len(c) - 1)]

    def _fit(self, tc, cx, cy, tm, vx, vy):
        dt = 0.01
        T0, T1 = max(tc[0], tm[0] - 2.0), min(tc[-1], tm[-1])
        if T1 - T0 < 5:
            return None
        grid = np.arange(T0, T1, dt)
        cxf, cyf = self._zoh(tc, cx, grid), self._zoh(tc, cy, grid)
        sel = (tm >= T0 + 1.0) & (tm <= T1)
        if sel.sum() < 15:
            return None
        tms, vxs, vys = tm[sel], vx[sel], vy[sel]
        best = None
        for L in np.arange(0.0, 1.01, 0.05):
            sh = int(round(L / dt))
            for tau in (0.02, 0.05, 0.1, 0.15, 0.25, 0.4, 0.7):
                a = 1 - math.exp(-dt / tau)
                K = int(min(len(grid), 8 * tau / dt + 2))
                ker = a * (1 - a) ** np.arange(K)
                ys = []
                for cf in (cxf, cyf):
                    cd = np.concatenate([np.full(sh, cf[0]), cf[:len(cf) - sh]])
                    cd = np.concatenate([np.full(K, cd[0]), cd])
                    y = np.convolve(cd, ker)[K:K + len(cf)]
                    ys.append(np.interp(tms, grid, y))
                A = np.zeros((2 * len(tms), 3))
                A[:len(tms), 0], A[len(tms):, 0] = ys[0], ys[1]
                A[:len(tms), 1] = 1.0
                A[len(tms):, 2] = 1.0
                b = np.concatenate([vxs, vys])
                coef, *_ = np.linalg.lstsq(A, b, rcond=None)
                sse = float(np.sum((b - A @ coef) ** 2))
                if best is None or sse < best[0]:
                    best = (sse, L, tau, coef)
        sse, L, tau, coef = best
        b = np.concatenate([vxs, vys])
        sst = float(np.sum((b - b.mean()) ** 2)) or 1e-12
        return {'lag': float(L), 'tau': float(tau), 'gain': float(coef[0]),
                'bias_x': float(coef[1]), 'bias_y': float(coef[2]),
                'r2': float(1 - sse / sst), 'rms_res': float(math.sqrt(sse / len(b)))}

    # ---- C7: oscillation --------------------------------------------------------
    def oscillation(self):
        r = self.r
        self.psd = {}
        if not self.window_ok:
            self.add('C7', 'oscillation of the drones', 'N/A', 'window too short', '')
            return
        rows, rms_trend = [], []
        for n in self.names:
            T = r.ticks[n]
            RX = r.rxs.get(n)
            if RX and len(RX.get('t', [])):
                t, vx, vy = RX['t'], RX['vx'], RX['vy']
            elif 'meas_vx' in T and np.isfinite(T['meas_vx']).any():
                t, vx, vy = T['t'], T['meas_vx'], T['meas_vy']
            else:   # legacy: only the command exists
                t, vx, vy = T['t'], T['cmd_x'], T['cmd_y']
            m = self.mask(t)
            t, vx, vy = t[m], vx[m], vy[m]
            ok = np.isfinite(vx) & np.isfinite(vy)
            t, vx, vy = t[ok], vx[ok], vy[ok]
            if len(t) < 40 or t[-1] - t[0] < 6:
                continue
            fs = 20.0
            tg = np.arange(t[0], t[-1], 1 / fs)
            sx, sy = np.interp(tg, t, vx), np.interp(tg, t, vy)
            # split into a slow part (3 s moving average = drift / slow breathing) and the
            # oscillation (everything faster than ~0.3 Hz)
            k = int(3 * fs)
            def movavg(a):
                p = np.pad(a, (k // 2, k - k // 2 - 1), mode='edge')
                return np.convolve(p, np.ones(k) / k, mode='valid')
            slow_x, slow_y = movavg(sx), movavg(sy)
            sx, sy = sx - slow_x, sy - slow_y
            slow_rms = float(math.sqrt(np.mean(slow_x ** 2 + slow_y ** 2)))
            w = np.hanning(len(tg))
            nfft = 1 << int(math.ceil(math.log2(len(tg) * 4)))
            P = np.abs(np.fft.rfft(sx * w, nfft)) ** 2 + np.abs(np.fft.rfft(sy * w, nfft)) ** 2
            f = np.fft.rfftfreq(nfft, 1 / fs)
            band = (f >= 0.3) & (f <= 8.0)
            if not band.any():
                continue
            ib = np.argmax(P[band])
            fpk = float(f[band][ib])
            prom = float(P[band][ib] / (np.median(P[band]) + 1e-18))
            rms = float(math.sqrt(np.mean(sx ** 2 + sy ** 2)))
            # growth: rms of the oscillation part in segments
            nseg = int(min(6, max(2, (tg[-1] - tg[0]) // 5)))
            edges = np.linspace(tg[0], tg[-1], nseg + 1)
            seg_rms, seg_t = [], []
            for a, b in zip(edges[:-1], edges[1:]):
                mm = (tg >= a) & (tg < b)
                if mm.sum() > 10:
                    seg_rms.append(math.sqrt(np.mean(sx[mm] ** 2 + sy[mm] ** 2)))
                    seg_t.append(0.5 * (a + b))
            slope = float('nan')
            if len(seg_rms) >= 3 and min(seg_rms) > 1e-6:
                slope = float(np.polyfit(seg_t, np.log(seg_rms), 1)[0])
            self.psd[n] = (f[band], P[band])
            rtf = self.rtf_eff
            self.per.setdefault(n, {}).update({
                'osc_hz_wall': fpk, 'osc_hz_sim': fpk / rtf if rtf else float('nan'),
                'osc_prom': prom, 'v_rms': rms, 'rms_slope': slope, 'slow_rms': slow_rms,
                'rms_first': seg_rms[0] if seg_rms else float('nan'),
                'rms_last': seg_rms[-1] if seg_rms else float('nan')})
            rows.append((n, fpk, prom, rms, slope, slow_rms))
        if not rows:
            self.add('C7', 'oscillation of the drones', 'N/A', 'not enough data', '')
            return
        rms_all = float(np.mean([x[3] for x in rows]))
        prom = float(np.mean([x[2] for x in rows]))
        fpk = float(np.median([x[1] for x in rows]))
        slopes = [x[4] for x in rows if math.isfinite(x[4])]
        slope = float(np.mean(slopes)) if slopes else float('nan')
        rtf = self.rtf_eff
        sustained = prom >= TH['osc_prom'] and rms_all >= TH['osc_rms']
        growing = math.isfinite(slope) and slope > TH['grow_slope'] and sustained
        if self.test_kind:      # the command itself is the test signal: not a controller oscillation
            sustained = growing = False
        decaying = math.isfinite(slope) and slope < -TH['grow_slope']
        slow = float(np.mean([x[5] for x in rows]))
        self.m['slow_drift_rms_mps'] = slow
        self.m.update({'v_rms_mps': rms_all, 'osc_hz_wall': fpk, 'osc_hz_sim': fpk / rtf,
                       'osc_prom': prom, 'rms_slope_per_s': slope,
                       'osc_growing': int(growing), 'osc_sustained': int(sustained)})
        st = 'FAIL' if growing else ('WARN' if sustained else 'PASS')
        if self.test_kind:
            st = 'INFO'
        word = 'GROWING' if growing else ('sustained' if sustained else ('decaying' if decaying else 'none/steady'))
        self.add('C7', 'oscillation of the drones', st,
                 f'oscillation (>0.3 Hz) rms {rms_all:.3f} m/s, dominant {fpk:.2f} Hz wall = {fpk / rtf:.2f} Hz sim '
                 f'(peak {prom:.0f}x median), amplitude trend {fnum(slope, 3)} /s -> {word}; '
                 f'slow part (<0.3 Hz, drift) rms {slow:.3f} m/s'
                 + ('' if (self.rtf_mean is not None or math.isfinite(self.m.get('rx_rate_hz', float('nan'))))
                    else ' [RTF unknown: "sim" = wall]')
                 + (' [test profile active: this is the test signal, not an instability]' if self.test_kind else ''),
                 f'sustained = peak > {TH["osc_prom"]}x median AND oscillation rms > {TH["osc_rms"]} m/s; growing = ln(rms) '
                 f'slope > {TH["grow_slope"]}/s')

    # ---- C8: swarm geometry / drift --------------------------------------------
    def geometry(self):
        r = self.r
        names = self.names
        # common time grid = control period bins
        t_all = [r.ticks[n]['t'] for n in names]
        tg = np.arange(self.t0, self.t1, self.period)
        X, Y = [], []
        for n in names:
            T = r.ticks[n]
            x = T['meas_x'] if np.isfinite(T.get('meas_x', np.array([np.nan]))).any() else T['used_x']
            y = T['meas_y'] if np.isfinite(T.get('meas_y', np.array([np.nan]))).any() else T['used_y']
            ok = np.isfinite(x) & np.isfinite(y)
            if not ok.any():
                continue
            X.append(np.interp(tg, T['t'][ok], x[ok]))
            Y.append(np.interp(tg, T['t'][ok], y[ok]))
        if len(X) < 1:
            self.add('C8', 'swarm geometry', 'N/A', 'no positions logged', '')
            return
        X, Y = np.array(X), np.array(Y)
        self.geo = {'t': tg, 'X': X, 'Y': Y}
        cx, cy = X.mean(0), Y.mean(0)
        rad = np.hypot(X - cx, Y - cy)
        mr = rad.mean(0)
        self.geo.update({'cx': cx, 'cy': cy, 'rad': rad, 'mr': mr})
        w = self.mask(tg)
        if w.sum() < 20:
            self.add('C8', 'swarm geometry', 'N/A', 'window too short', '')
            return
        # pairwise nearest distance
        if len(X) > 1:
            dmin = np.full(len(tg), np.inf)
            for i in range(len(X)):
                for j in range(i + 1, len(X)):
                    dmin = np.minimum(dmin, np.hypot(X[i] - X[j], Y[i] - Y[j]))
            self.geo['dmin'] = dmin
        k = max(5, int(1.0 / self.period))
        def avg(a, s):    # mean over first/last second of the window
            idx = np.where(w)[0]
            sel = idx[:k] if s == 0 else idx[-k:]
            return float(np.mean(a[sel]))
        rad0, rad1 = avg(mr, 0), avg(mr, 1)
        span = tg[w][-1] - tg[w][0]
        rslope = (rad1 - rad0) / span if span > 0 else float('nan')
        cdrift = math.hypot(avg(cx, 1) - avg(cx, 0), avg(cy, 1) - avg(cy, 0))
        cspeed = cdrift / span if span > 0 else float('nan')
        per_disp = {}
        for i, n in enumerate(names[:len(X)]):
            idx = np.where(w)[0]
            dx = float(np.mean(X[i][idx[-k:]]) - np.mean(X[i][idx[:k]]))
            dy = float(np.mean(Y[i][idx[-k:]]) - np.mean(Y[i][idx[:k]]))
            per_disp[n] = (dx, dy)
            self.per.setdefault(n, {})['disp_m'] = math.hypot(dx, dy)
        self.m.update({'ring_radius_start_m': rad0, 'ring_radius_end_m': rad1,
                       'ring_radius_trend_mps': rslope, 'centroid_drift_m': cdrift,
                       'centroid_speed_mps': cspeed,
                       'max_displacement_m': max(math.hypot(*v) for v in per_disp.values())})
        eq = float('nan')
        if self.sph and self.n >= 2:
            eq = equilibrium_ring_radius(self.n, self.sph.get('mass', 7000.0), self.sph.get('h', 3.0),
                                         self.sph.get('rho_0', 1000.0))
        self.m['ring_radius_equilibrium_m'] = eq
        txt = (f'mean radius from centroid {rad0:.2f} -> {rad1:.2f} m over {span:.0f} s '
               f'(trend {rslope * 1000:+.1f} mm/s); equilibrium (rho=rho0) ring radius {fnum(eq)} m; '
               f'centroid moved {cdrift * 100:.0f} cm ({cspeed * 1000:.1f} mm/s)')
        st = 'PASS'
        if abs(rslope) > 0.01 or cspeed > TH['drift_warn']:
            st = 'WARN'
        if abs(rslope) > 0.03 or cspeed > TH['drift_fail']:
            st = 'FAIL'
        mx = self.m['max_displacement_m']
        if not self.test_kind:
            who = max(per_disp, key=lambda q: math.hypot(*per_disp[q]))
            txt += f'; largest displacement {mx:.2f} m ({who})'
            if mx > 0.4:
                st = 'FAIL'
            elif mx > 0.15 and st == 'PASS':
                st = 'WARN'
        if self.test_kind:
            st = 'INFO'
            txt += ' [test profile moves the swarm on purpose; judge only the radius trend]'
        self.add('C8', 'swarm shape and drift', st, txt,
                 'radius trend and centroid speed should be ~0 when the swarm is at rest')
        if 'dmin' in self.geo:
            dm = self.geo['dmin'][w]
            self.m['min_pair_dist_m'] = float(np.min(dm))
            st = 'PASS' if np.min(dm) > 0.25 else ('WARN' if np.min(dm) > 0.15 else 'FAIL')
            self.add('C9', 'closest approach between drones', st,
                     f'min {np.min(dm):.2f} m, median {np.median(dm):.2f} m',
                     '> 0.25 m ok (Crazyflie ~0.1 m across, downwash)')

    # ---- C10: SPH regime ---------------------------------------------------------
    def sph_regime(self):
        r = self.r
        rho0 = self.sph.get('rho_0') or 1000.0
        oks, rmin, rmax, pmin, pmax = [], [], [], [], []
        for n in self.names:
            T = r.ticks[n]
            m = self.mask(T['t'])
            rho = T['rho'][m]
            if not len(rho):
                continue
            oks.append(100.0 * np.mean(np.abs(rho / rho0 - 1) <= TH['rho_band']))
            rmin.append(float(np.min(rho))); rmax.append(float(np.max(rho)))
            pmin.append(float(np.min(T['P'][m]))); pmax.append(float(np.max(T['P'][m])))
            self.per.setdefault(n, {}).update({'rho_mean': float(np.mean(rho)),
                                               'rho_min': float(np.min(rho)), 'rho_max': float(np.max(rho)),
                                               'P_mean': float(np.mean(T['P'][m]))})
        if not oks:
            return
        ok = float(np.mean(oks))
        self.m.update({'rho_within_10pct': ok, 'rho_min': min(rmin), 'rho_max': max(rmax),
                       'P_min': min(pmin), 'P_max': max(pmax)})
        st = 'PASS' if ok >= TH['rho_ok_warn'] else 'WARN'
        # startup transient (before the window) -- from the whole log
        t_ok = []
        for n in self.names:
            T = r.ticks[n]
            good = np.abs(T['rho'] / rho0 - 1) <= 0.2
            idx = np.where(good)[0]
            if len(idx):
                t_ok.append(float(T['t'][idx[0]]))
        trans = f'; first within 20 % of rho0 at t={max(t_ok):.1f} s' if t_ok else ''
        self.add('C10', 'SPH density regime', st,
                 f'rho within +-10 % of rho0 {ok:.0f} % of the window; rho {min(rmin):.0f}..{max(rmax):.0f}, '
                 f'P {min(pmin):.0f}..{max(pmax):.0f}{trans}',
                 f'>= {TH["rho_ok_warn"]:.0f} % in band. Large |P| means SPH is pushing hard; rho << rho0 = '
                 f'drones too far apart or neighbours missing')

    # ---- C11: force budget -----------------------------------------------------
    def forces(self):
        r = self.r
        keys = [('ap', 'pressure'), ('av', 'viscosity'), ('ad', 'damping'), ('ae', 'external'), ('a', 'total')]
        tab = {}
        for n in self.names:
            T = r.ticks[n]
            if 'ap_x' not in T:
                continue
            m = self.mask(T['t'])
            row = {}
            for k, _ in keys:
                v = np.hypot(T[f'{k}_x'][m], T[f'{k}_y'][m])
                row[k] = float(math.sqrt(np.mean(v ** 2))) if len(v) else float('nan')
            tab[n] = row
            self.per.setdefault(n, {}).update({f'a_{k}_rms': row[k] for k, _ in keys})
        if not tab:
            self.add('C11', 'acceleration budget', 'N/A', 'old log format has no force split', '')
            return
        mean = {k: float(np.mean([t[k] for t in tab.values()])) for k, _ in keys}
        self.m.update({f'a_{k}_rms': v for k, v in mean.items()})
        dom = max(((k, nm) for k, nm in keys if k != 'a'), key=lambda kn: mean[kn[0]])[1]
        self.add('C11', 'acceleration budget (rms, m/s^2)', 'INFO',
                 f'pressure {mean["ap"]:.3f}, viscosity {mean["av"]:.3f}, damping {mean["ad"]:.3f}, '
                 f'external {mean["ae"]:.3f}, total {mean["a"]:.3f} -> dominant: {dom}',
                 'tells which term actually moves the drones')
        self.forces_tab = tab

    # ---- C12: command health ----------------------------------------------------
    def command(self):
        r = self.r
        clip, chat = [], []
        for n in self.names:
            T = r.ticks[n]
            m = self.mask(T['t'])
            if 'clipped' in T:
                clip.append(100.0 * np.mean(T['clipped'][m] > 0))
            cx, cy = T['cmd_raw_x'][m], T['cmd_raw_y'][m]
            ok = np.isfinite(cx) & np.isfinite(cy)
            cx, cy = cx[ok], cy[ok]
            if len(cx) > 10:
                big = (np.hypot(cx[1:], cy[1:]) > 0.02) & (np.hypot(cx[:-1], cy[:-1]) > 0.02)
                dots = cx[1:] * cx[:-1] + cy[1:] * cy[:-1]
                flips = (dots < 0) & big
                chat.append(100.0 * flips.sum() / max(big.sum(), 1))
                self.per.setdefault(n, {})['flip_pct'] = chat[-1]
        if clip:
            c = float(np.mean(clip))
            self.m['clip_pct'] = c
            st = 'PASS' if c < TH['clip_warn'] else ('WARN' if c < TH['clip_fail'] else 'FAIL')
            self.add('C12', 'speed command saturation', st,
                     f'{c:.1f} % of ticks clipped at {self.max_v} m/s',
                     f'< {TH["clip_warn"]} % ok. The un-clipped value keeps integrating in stale ticks')
        if chat:
            c = float(np.mean(chat))
            self.m['cmd_flip_pct'] = c
            st = 'PASS' if c < TH['chatter_warn'] else ('WARN' if c < TH['chatter_fail'] else 'FAIL')
            self.add('C13', 'command chatter (direction flips tick to tick)', st,
                     f'{c:.0f} % of consecutive ticks reverse direction',
                     f'< {TH["chatter_warn"]} % ok. High = damping gain too big for the step (zeta * dt > 1)')

    # ---- C14: altitude ----------------------------------------------------------
    def altitude(self):
        r = self.r
        dz, zs = [], []
        for n in self.names:
            T = r.ticks[n]
            if 'meas_z' not in T or not np.isfinite(T['meas_z']).any():
                continue
            m = self.mask(T['t'])
            z = T['meas_z'][m]
            if len(z) < 10:
                continue
            t = T['t'][m]
            slope = float(np.polyfit(t, z, 1)[0])
            dz.append(abs(float(np.mean(z)) - self.fixed_z))
            zs.append((float(np.mean(z)), float(np.std(z)), slope, float(np.sqrt(np.mean(T['meas_vz'][m] ** 2)))))
            self.per.setdefault(n, {}).update({'z_mean': zs[-1][0], 'z_std': zs[-1][1],
                                               'z_slope': zs[-1][2], 'vz_rms': zs[-1][3]})
        if not dz:
            self.add('C14', 'altitude', 'N/A', 'no z logged', '')
            return
        worst = max(dz)
        slope = max(abs(z[2]) for z in zs)
        self.m.update({'z_err_max_m': worst, 'z_slope_max_mps': slope})
        st = 'PASS' if worst < TH['z_warn'] and slope < 0.01 else ('WARN' if worst < TH['z_fail'] else 'FAIL')
        self.add('C14', 'altitude hold (z, commanded vz = 0)', st,
                 f'z mean {np.mean([z[0] for z in zs]):.2f} m (target {self.fixed_z}), worst error '
                 f'{worst * 100:.0f} cm, worst slope {slope * 1000:.1f} mm/s',
                 f'< {TH["z_warn"] * 100:.0f} cm error and < 10 mm/s slope')

    # ---- events -----------------------------------------------------------------
    def events(self):
        counts = {}
        for ln in self.r.events:
            m = re.match(r'\s*[\d.-]+\s+(INFO|WARN|ERROR)\s+\S+\s+(\S+)', ln)
            if m:
                counts[(m.group(1), m.group(2))] = counts.get((m.group(1), m.group(2)), 0) + 1
        self.ev_counts = counts

    # ---- diagnosis --------------------------------------------------------------
    def diagnose(self):
        m = self.m
        d = []
        rtf = self.rtf_eff
        if self.diverged_at is not None:
            d.append(('HIGH', 'DIVERGED',
                      f'The run blew up at t = {self.diverged_at:.1f} s ({self.diverged_why}; physically impossible '
                      f'for a drone commanded <= 0.5 m/s: crash or estimator divergence). Numbers below use only '
                      f't < {self.w1:.1f} s. Look at fig_overview.png and the first WARN lines of events.log '
                      f'before that time: that is where the cause is.'))
        osc = m.get('osc_sustained', 0)
        grow = m.get('osc_growing', 0)
        if self.sim_clock:
            d.append(('INFO', 'TIME-BASE-OK',
                      f'sim_clock=gz: the controller ticked every {self.m.get("tick_dt_p50_ms", float("nan")):.0f} ms '
                      f'of SIMULATED time (p95 {self.m.get("tick_dt_p95_ms", float("nan")):.0f} ms) and integrated '
                      f'dt={self.dt_model * 1000:.0f} ms per tick, so the time-base mismatch is gone at RTF {rtf:.2f}. '
                      f'Low RTF now only makes the run slow, not different.'))
        elif rtf < TH['rtf_warn']:
            sev = 'HIGH' if osc else 'MED'
            d.append((sev, 'TIME-BASE',
                      f'The simulator runs at about {rtf:.2f}x real time while the controller ticks at 20 Hz of '
                      f'WALL time: each control tick covers only {rtf * self.period * 1000:.0f} ms of simulated '
                      f'time instead of {self.period * 1000:.0f} ms, and the 20 Hz state topic arrives at '
                      f'about {m.get("rx_rate_hz", float("nan")):.1f} Hz wall. The SPH model integrates '
                      f'dt={self.dt_model * 1000:.0f} ms per tick, but only {rtf * self.period * 1000:.0f} ms of '
                      f'simulated time passed, so per SIMULATED second the controller applies '
                      f'{1 / max(rtf, 1e-3):.1f}x the acceleration the SPH model intends. '
                      f'Test T6/T7 (RTF sweep, dt_model scaled by RTF).'))
        if m.get('stale_pct', 0) > TH['stale_warn']:
            d.append(('MED', 'STALE-STATE',
                      f'{m["stale_pct"]:.0f} % of ticks have no new state message; with stale_policy=predict '
                      f'those ticks integrate the controller\'s own prediction. Compare stale_policy=hold (T8).'))
        lag = m.get('track_lag_sim_s')
        if lag is not None and (lag > TH['lag_warn'] or m.get('track_r2', 1) < TH['r2_warn'] or
                                not TH['gain_lo'] <= m.get('track_gain', 1) <= TH['gain_hi']):
            d.append(('HIGH', 'DRONE-TRACKING',
                      f'The drone does not follow velocity commands like an ideal integrator: '
                      f'gain {m.get("track_gain", float("nan")):.2f}, total lag {lag * 1000:.0f} ms (sim), '
                      f'R2 {m.get("track_r2", float("nan")):.2f}. Any controller closed around this has less '
                      f'phase margin; compare with T3 (step/chirp test, no SPH).'))
        if m.get('hover_drift_mps', 0) > TH['drift_warn']:
            d.append(('HIGH' if m['hover_drift_mps'] > TH['drift_fail'] else 'MED', 'OWN-DRIFT',
                      f'When commanded ~0 the drones still move at up to {m["hover_drift_mps"] * 1000:.0f} mm/s. '
                      f'That is firmware / estimator / simulator behaviour, not SPH. Test T2 (zero command) '
                      f'isolates it.'))
        if grow:
            L = (m.get('track_lag_sim_s') or 0.0)
            f_sim = m['osc_hz_sim']
            hint = ''
            if L > 0:
                f_pred = 1.0 / (4.0 * (L + 0.5 * self.period * rtf))
                hint = (f' Delay-induced instability would oscillate near 1/(4 x lag) = {f_pred:.2f} Hz sim; '
                        f'observed {f_sim:.2f} Hz sim.')
            d.append(('HIGH', 'GROWING-OSCILLATION',
                      f'Velocity amplitude grows ({m["rms_slope_per_s"]:+.3f} per s on ln rms) at {f_sim:.2f} Hz '
                      f'sim ({m["osc_hz_wall"]:.2f} Hz wall): the loop is unstable.{hint} '
                      f'Find which part causes it with T1 (damping only) -> T4 (SPH only).'))
        elif osc:
            d.append(('MED', 'SUSTAINED-OSCILLATION',
                      f'A steady oscillation at {m["osc_hz_sim"]:.2f} Hz sim ({m["osc_hz_wall"]:.2f} Hz wall), '
                      f'rms {m["v_rms_mps"]:.3f} m/s, is present but not growing.'))
        if m.get('clip_pct', 0) > TH['clip_warn']:
            d.append(('MED', 'SATURATION', f'{m["clip_pct"]:.0f} % of ticks hit the speed limit.'))
        if m.get('cmd_flip_pct', 0) > TH['chatter_warn']:
            d.append(('MED', 'CHATTER', f'{m["cmd_flip_pct"]:.0f} % of ticks flip the command direction: '
                      f'damping zeta*dt is probably too close to or above 1 (zeta={self.sph.get("zetta_extra")}, '
                      f'dt={self.dt_model}).'))
        # static design checks from meta
        z = self.sph.get('zetta_extra')
        if z is not None and z * self.dt_model >= 1.0:
            d.append(('HIGH', 'DESIGN', f'zeta * dt_model = {z * self.dt_model:.2f} >= 1: the damping term '
                      f'alone reverses the velocity every tick.'))
        eq = m.get('ring_radius_equilibrium_m')
        r0 = m.get('ring_radius_start_m')
        if eq and r0 and math.isfinite(eq) and abs(r0 - eq) / eq > 0.15 and \
                (self.params.get('f_k', 0) in (0, 0.0)):
            d.append(('INFO', 'GEOMETRY', f'Ring radius {r0:.2f} m differs from the equilibrium radius '
                      f'{eq:.2f} m (rho = rho0): pressure is not zero at rest, SPH will push the ring '
                      f'{"outward" if r0 < eq else "inward"}.'))
        if not d:
            d.append(('OK', 'NONE', 'No check flagged a problem in this run.'))
        order = {'HIGH': 0, 'MED': 1, 'INFO': 2, 'OK': 3}
        d.sort(key=lambda x: order.get(x[0], 9))
        self.diag = d

    def run_all(self):
        self.timing()
        self.tracking()
        self.mean_motion()
        self.oscillation()
        self.geometry()
        self.sph_regime()
        self.forces()
        self.command()
        self.altitude()
        self.events()
        self.diagnose()
        self.checks.sort(key=lambda c: int(c[0][1:]))


# ----------------------------------------------------------------------------- output
STATUS_ORDER = {'FAIL': 0, 'WARN': 1, 'PASS': 3, 'INFO': 2, 'N/A': 4}


def write_outputs(A, outdir):
    os.makedirs(outdir, exist_ok=True)
    meta = A.r.meta or {}
    label = meta.get('label', '')
    P = A.params
    head = [
        f'run: {A.r.name}   label: {label}   code: {meta.get("git_rev", "?")}   '
        f'drones: {A.n}   mode: {meta.get("node", "?")}',
        f'window analysed: t = {A.w0:.1f} .. {A.w1:.1f} s '
        f'({"after flight start at %.1f s" % A.flight_start if A.flight_start is not None else "NO FLIGHT DETECTED - whole run"}'
        f'{", test profile: " + A.test_kind if A.test_kind else ""})',
        f'params: sph_gain={P.get("sph_gain", "?")} zeta={A.sph.get("zetta_extra", "?")} '
        f'gamma={A.sph.get("gamma", "?")} mass={A.sph.get("mass", "?")} f_k={P.get("f_k", "?")} '
        f'dt_model={A.dt_model} policy={(meta.get("core") or {}).get("stale_policy", "?")} '
        f'gate={(meta.get("core") or {}).get("start_gate", "?")} '
        f'clock={(meta.get("core") or {}).get("sim_clock", "wall")}',
    ]
    if A.r.legacy:
        head.append('OLD LOG FORMAT: only density, pressure, command and position exist; most checks are N/A. '
                    'Rerun with the new logging for the full analysis.')
    # ------------------------------------------------ summary.txt
    lines = ['SPH RUN SUMMARY'] + head + ['']
    lines.append('VERDICTS')
    for cid, title, st, val, rule in sorted(A.checks, key=lambda c: (STATUS_ORDER.get(c[2], 9), int(c[0][1:]))):
        lines.append(f'  [{st:<4}] {cid:<3} {title}: {val}')
    lines.append('')
    lines.append('DIAGNOSIS (most important first)')
    for sev, code, text in A.diag:
        lines.append(f'  [{sev}] {code}: {text}')
    ev = sorted(A.ev_counts.items(), key=lambda kv: -kv[1])
    if ev:
        lines.append('')
        lines.append('EVENTS: ' + ', '.join(f'{c}x{n} ({l})' for (l, c), n in ev if l != 'INFO' or c in ('GATE_OPEN',)))
    with open(os.path.join(outdir, 'summary.txt'), 'w') as f:
        f.write('\n'.join(lines) + '\n')

    # ------------------------------------------------ report.md
    md = [f'# SPH run report: {A.r.name}', ''] + [f'- {h}' for h in head] + ['']
    md += ['## Diagnosis', '']
    for sev, code, text in A.diag:
        md.append(f'- **[{sev}] {code}** - {text}')
    md += ['', '## Health checks', '', '| id | check | result | measured | threshold / meaning |', '|---|---|---|---|---|']
    for cid, title, st, val, rule in A.checks:
        md.append(f'| {cid} | {title} | **{st}** | {val} | {rule} |')
    md += ['', '## Per drone', '']
    cols = [('rx_hz', 'rx Hz', 1), ('stale_pct', 'stale %', 0), ('pred_err_p95', 'pred err p95 m', 3),
            ('gain', 'gain', 2), ('delay_ms', 'delay ms', 0), ('tau_ms', 'tau ms', 0), ('r2', 'R2', 2),
            ('v_rms', 'osc rms m/s', 3), ('slow_rms', 'slow rms m/s', 3), ('osc_hz_sim', 'osc Hz sim', 2), ('rms_slope', 'rms slope /s', 3),
            ('rho_mean', 'rho mean', 0), ('P_mean', 'P mean', 0), ('a_ap_rms', 'a press', 3),
            ('a_ad_rms', 'a damp', 3), ('a_ae_rms', 'a ext', 3), ('disp_m', 'moved m', 2),
            ('hover_vx', 'hover vx', 3), ('hover_vy', 'hover vy', 3), ('z_mean', 'z', 2), ('flip_pct', 'flip %', 0)]
    cols = [c for c in cols if any(c[0] in A.per.get(n, {}) for n in A.names)]
    md.append('| drone | ' + ' | '.join(c[1] for c in cols) + ' |')
    md.append('|---|' + '---|' * len(cols))
    for n in A.names:
        md.append(f'| {n} | ' + ' | '.join(fnum(A.per.get(n, {}).get(k, float('nan')), nd) for k, _, nd in cols) + ' |')
    if getattr(A, 'ev_counts', None):
        md += ['', '## Events (events.log)', '', '| level | code | count |', '|---|---|---|']
        for (l, c), n in sorted(A.ev_counts.items(), key=lambda kv: (kv[0][0] != 'WARN', -kv[1])):
            md.append(f'| {l} | {c} | {n} |')
    md += ['', '## Figures', '']
    md += [f'- `{f}`' for f in sorted(os.listdir(outdir)) if f.endswith('.png')]
    md += ['', '## Glossary', '',
           '- **stale tick**: the control timer fired but no new state message from that drone arrived since the '
           'previous tick.',
           '- **RTF**: Gazebo real-time factor = simulated seconds per wall second. Control ticks use wall time.',
           '- **hover drift**: mean measured velocity on ticks where the command was ~0.',
           '- **tau / delay**: first-order time constant and pure delay fitted between command and measured '
           'velocity (wall seconds; multiplied by RTF to get sim seconds).',
           '- **a press / damp / ext**: rms of the pressure, damping (-zeta v) and external-force parts of dv/dt.',
           '- full column list of the raw logs: see sph_diag.py (TICK_COLUMNS) or TEST_GUIDE.md section 3.']
    with open(os.path.join(outdir, 'report.md'), 'w') as f:
        f.write('\n'.join(md) + '\n')
    with open(os.path.join(outdir, 'report.json'), 'w') as f:
        json.dump({'run': A.r.name, 'label': label, 'metrics': A.m, 'per_drone': A.per,
                   'checks': [{'id': c[0], 'title': c[1], 'status': c[2], 'value': c[3]} for c in A.checks],
                   'diagnosis': [{'severity': s, 'code': c, 'text': t} for s, c, t in A.diag],
                   'window': [A.w0, A.w1], 'flight_start': A.flight_start}, f, indent=2, default=str)
    return os.path.join(outdir, 'summary.txt')


# ----------------------------------------------------------------------------- figures
def figures(A, outdir):
    os.makedirs(outdir, exist_ok=True)
    r = A.r
    cols = plt.rcParams['axes.prop_cycle'].by_key()['color']
    C = {n: cols[i % len(cols)] for i, n in enumerate(A.names)}
    rho0 = A.sph.get('rho_0') or 1000.0

    def shade(ax):
        ax.axvspan(A.w0, A.w1, color='0.5', alpha=0.08, lw=0)
        ax.grid(True, alpha=0.3)

    # 1 overview
    fig, ax = plt.subplots(6, 1, figsize=(11, 15), sharex=True)
    for n in A.names:
        T = r.ticks[n]
        t = T['t']
        v = r.rxs.get(n)
        if v and len(v.get('t', [])):
            ax[0].plot(v['t'], np.hypot(v['vx'], v['vy']), color=C[n], lw=1, label=n)
        elif 'meas_vx' in T and np.isfinite(T['meas_vx']).any():
            ax[0].plot(t, np.hypot(T['meas_vx'], T['meas_vy']), color=C[n], lw=1, label=n)
        ax[1].plot(t, np.hypot(T['cmd_raw_x'], T['cmd_raw_y']), color=C[n], lw=1, label=n)
        ax[2].plot(t, T['rho'] / rho0, color=C[n], lw=1)
        if 'meas_z' in T:
            ax[5].plot(t, T['meas_z'], color=C[n], lw=1)
    ax[0].set_ylabel('|measured v| (m/s)')
    ax[1].set_ylabel('|command| (m/s)')
    ax[1].axhline(A.max_v, color='r', ls=':', lw=1)
    ax[2].set_ylabel('rho / rho0')
    ax[2].axhspan(1 - TH['rho_band'], 1 + TH['rho_band'], color='g', alpha=0.1)
    if hasattr(A, 'geo'):
        ax[3].plot(A.geo['t'], A.geo['mr'], 'k', lw=1.2, label='mean radius')
        for i, n in enumerate(A.names[:len(A.geo['rad'])]):
            ax[3].plot(A.geo['t'], A.geo['rad'][i], color=C[n], lw=0.8, alpha=0.7)
        eq = A.m.get('ring_radius_equilibrium_m')
        if eq and math.isfinite(eq):
            ax[3].axhline(eq, color='g', ls='--', lw=1, label='equilibrium radius')
        ax[3].legend(fontsize=8)
    ax[3].set_ylabel('radius from centroid (m)')
    for n in A.names:
        T = r.ticks[n]
        if 'rx_since' in T:
            k = 40                                  # 2 s of ticks
            st_ = (T['rx_since'] == 0).astype(float)
            if len(st_) > k:
                roll = np.convolve(st_, np.ones(k) / k, mode='valid') * 100
                ax[4].plot(T['t'][k - 1:], roll, color=C[n], lw=1)
    ax[4].set_ylabel('stale ticks, 2 s average (%)')
    ax[4].set_ylim(0, 100)
    ax[5].set_ylabel('z (m)')
    ax[5].set_xlabel('time (s)')
    if A.rtf_series is not None:
        a2 = ax[4].twinx()
        a2.plot(A.rtf_series[0], A.rtf_series[1], 'k--', lw=1)
        a2.set_ylabel('RTF (dashed)')
        a2.set_ylim(0, max(1.1, float(np.nanmax(A.rtf_series[1])) * 1.1))
    for a in ax:
        shade(a)
    ax[0].legend(ncol=min(6, A.n), fontsize=8)
    ax[0].set_title(f'{A.r.name}: shaded = analysed window')
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, 'fig_overview.png'), dpi=110)
    plt.close(fig)

    # 2 tracking: command vs measured v for each drone (x axis), + fitted model
    if not r.legacy and r.rxs:
        n_show = A.names[:min(3, A.n)]
        fig, axs = plt.subplots(len(n_show), 1, figsize=(11, 3.2 * len(n_show)), sharex=True, squeeze=False)
        for a, n in zip(axs[:, 0], n_show):
            T, RX = r.ticks[n], r.rxs.get(n)
            if not RX:
                continue
            a.step(T['t'], T['cmd_x'], where='post', color='tab:red', lw=1, label='command vx')
            a.plot(RX['t'], RX['vx'], color='tab:blue', lw=1, label='measured vx')
            a.step(T['t'], T['cmd_y'], where='post', color='tab:orange', lw=1, alpha=0.7, label='command vy')
            a.plot(RX['t'], RX['vy'], color='tab:cyan', lw=1, alpha=0.7, label='measured vy')
            a.set_ylabel(f'{n}  v (m/s)')
            shade(a)
        axs[0, 0].legend(ncol=4, fontsize=8)
        axs[-1, 0].set_xlabel('time (s)')
        axs[0, 0].set_title('command vs measured velocity (lag / gain / drift are visible here)')
        fig.tight_layout()
        fig.savefig(os.path.join(outdir, 'fig_tracking.png'), dpi=110)
        plt.close(fig)

    # 3 spectrum
    if A.psd:
        fig, (a1, a2) = plt.subplots(1, 2, figsize=(12, 4.2))
        rtf = A.rtf_eff
        for n, (f, Pw) in A.psd.items():
            a1.semilogy(f, Pw, color=C[n], lw=1, label=n)
            a2.semilogy(f / rtf, Pw, color=C[n], lw=1)
        a1.set_xlabel('frequency (Hz, wall time)')
        a2.set_xlabel(f'frequency (Hz, sim time, RTF {rtf:.2f})')
        a1.set_ylabel('velocity power')
        a1.legend(fontsize=8)
        for a in (a1, a2):
            a.grid(True, alpha=0.3, which='both')
        fig.suptitle('Spectrum of measured velocity in the analysed window')
        fig.tight_layout()
        fig.savefig(os.path.join(outdir, 'fig_spectrum.png'), dpi=110)
        plt.close(fig)

    # 4 forces
    if hasattr(A, 'forces_tab'):
        fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 4.5), gridspec_kw={'width_ratios': [1, 1.6]})
        keys = [('ap', 'pressure'), ('av', 'viscosity'), ('ad', 'damping'), ('ae', 'external')]
        w = 0.8 / len(keys)
        for j, (k, lab) in enumerate(keys):
            a1.bar(np.arange(len(A.forces_tab)) + j * w, [t[k] for t in A.forces_tab.values()], w, label=lab)
        a1.set_xticks(np.arange(len(A.forces_tab)) + 0.4 - w / 2)
        a1.set_xticklabels(list(A.forces_tab))
        a1.set_ylabel('rms acceleration (m/s^2)')
        a1.legend(fontsize=8)
        a1.grid(True, alpha=0.3)
        n0 = A.names[0]
        T = r.ticks[n0]
        for k, lab in keys:
            a2.plot(T['t'], np.hypot(T[f'{k}_x'], T[f'{k}_y']), lw=1, label=lab)
        a2.set_yscale('symlog', linthresh=1e-3)
        a2.set_title(f'{n0}: magnitude of each term over time')
        a2.legend(fontsize=8)
        shade(a2)
        a2.set_xlabel('time (s)')
        fig.tight_layout()
        fig.savefig(os.path.join(outdir, 'fig_forces.png'), dpi=110)
        plt.close(fig)

    # 5 trajectory
    if hasattr(A, 'geo'):
        fig, ax1 = plt.subplots(figsize=(7, 7))
        for i, n in enumerate(A.names[:len(A.geo['X'])]):
            ax1.plot(A.geo['X'][i], A.geo['Y'][i], color=C[n], lw=1.2, label=n)
            ax1.plot(A.geo['X'][i][0], A.geo['Y'][i][0], 'o', color=C[n], mec='k')
            ax1.plot(A.geo['X'][i][-1], A.geo['Y'][i][-1], 's', color=C[n], mec='k')
        th = np.linspace(0, 2 * math.pi, 200)
        eq = A.m.get('ring_radius_equilibrium_m')
        cx, cy = A.geo['cx'][0], A.geo['cy'][0]
        if eq and math.isfinite(eq):
            ax1.plot(cx + eq * np.cos(th), cy + eq * np.sin(th), 'g--', lw=1, label='equilibrium ring')
        xc, yc, R = A.params.get('xc'), A.params.get('yc'), A.params.get('R')
        if xc is not None and A.params.get('f_k', 0):
            ax1.plot(xc + R * np.cos(th), yc + R * np.sin(th), 'k:', lw=1, label='potential circle')
        ax1.set_aspect('equal')
        ax1.grid(True, alpha=0.3)
        ax1.legend(fontsize=8)
        ax1.set_title('paths (o start, square end)')
        fig.tight_layout()
        fig.savefig(os.path.join(outdir, 'fig_trajectory.png'), dpi=110)
        plt.close(fig)

    # 6 timing
    if not r.legacy:
        fig, ax = plt.subplots(1, 3, figsize=(14, 3.8))
        col = 'sim_dt_ms' if A.sim_clock and all('sim_dt_ms' in r.ticks[n] for n in A.names) else 'wall_dt_ms'
        wd = np.concatenate([r.ticks[n][col] for n in A.names])
        wd = wd[np.isfinite(wd)]
        ax[0].hist(wd, bins=60, color='tab:blue')
        ax[0].axvline(A.period * 1000, color='k', ls='--')
        ax[0].set_xlabel('time between control ticks (ms, %s)' % ('simulated' if col == 'sim_dt_ms' else 'wall'))
        rd = np.concatenate([r.rxs[n]['rx_dt_ms'] for n in r.rxs if len(r.rxs[n].get('t', []))] or [np.array([])])
        rd = rd[np.isfinite(rd)]
        if len(rd):
            ax[1].hist(rd, bins=60, color='tab:orange')
            ax[1].axvline(1000 / STATE_HZ_NOMINAL, color='k', ls='--', label='nominal 20 Hz')
            ax[1].legend(fontsize=8)
        ax[1].set_xlabel('time between state messages (ms, wall)')
        sr = np.concatenate([r.ticks[n]['rx_since'] for n in A.names])
        ax[2].hist(sr, bins=np.arange(-0.5, max(4, sr.max() + 1.5)), color='tab:green', rwidth=0.8)
        ax[2].set_xlabel('state messages received per control tick')
        ax[2].set_yscale('log')
        for a in ax:
            a.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(outdir, 'fig_timing.png'), dpi=110)
        plt.close(fig)


# ----------------------------------------------------------------------------- compare
COMPARE_ROWS = [
    ('label', 'label', None), ('n', 'drones', 0),
    ('rtf_used', 'RTF', 2), ('stale_pct', 'stale %', 0), ('late_pct', 'late ticks %', 1),
    ('track_gain', 'track gain', 2), ('track_lag_sim_s', 'lag sim s', 2), ('track_r2', 'track R2', 2),
    ('hover_drift_mps', 'hover drift m/s', 3), ('v_rms_mps', 'v rms m/s', 3),
    ('osc_hz_sim', 'osc Hz sim', 2), ('rms_slope_per_s', 'growth /s', 3), ('osc_growing', 'growing', 0),
    ('ring_radius_trend_mps', 'radius trend m/s', 3), ('centroid_speed_mps', 'centroid m/s', 3),
    ('max_displacement_m', 'max moved m', 2), ('rho_within_10pct', 'rho ok %', 0),
    ('a_ap_rms', 'a press', 3), ('a_ad_rms', 'a damp', 3), ('a_ae_rms', 'a ext', 3),
    ('clip_pct', 'clip %', 1), ('cmd_flip_pct', 'chatter %', 0), ('min_pair_dist_m', 'min dist m', 2),
]


def compare(paths, out):
    results = []
    for p in paths:
        run = Run(p)
        A = Analysis(run)
        A.run_all()
        results.append((run, A))
    md = ['# Run comparison', '', '| metric | ' + ' | '.join(r.name[:34] for r, _ in results) + ' |',
          '|---|' + '---|' * len(results)]
    for key, lab, nd in COMPARE_ROWS:
        cells = []
        for run, A in results:
            if key == 'label':
                cells.append(str((run.meta or {}).get('label', '')))
            elif key == 'n':
                cells.append(str(A.n))
            else:
                cells.append(fnum(A.m.get(key, float('nan')), nd))
        md.append(f'| {lab} | ' + ' | '.join(cells) + ' |')
    md += ['', '## Top diagnosis per run', '']
    for run, A in results:
        md.append(f'- **{run.name}**: ' + '; '.join(f'[{s}] {c}' for s, c, _ in A.diag[:3]))
    os.makedirs(out, exist_ok=True)
    path = os.path.join(out, 'compare.md')
    with open(path, 'w') as f:
        f.write('\n'.join(md) + '\n')
    # overlay: speed rms over time
    fig, ax = plt.subplots(2, 1, figsize=(11, 7), sharex=False)
    for run, A in results:
        lab = (run.meta or {}).get('label') or run.name
        ts, rs = [], []
        for n in A.names:
            RX = run.rxs.get(n)
            if RX and len(RX.get('t', [])):
                ts.append(RX['t'])
        if ts and not run.legacy:
            tg = np.arange(A.w0, A.w1, 1.0)
            vals = []
            for tt in tg:
                acc = []
                for n in A.names:
                    RX = run.rxs.get(n)
                    if RX and len(RX['t']):
                        m = (RX['t'] >= tt) & (RX['t'] < tt + 1.0)
                        if m.sum() > 2:
                            acc.append(np.mean(RX['vx'][m] ** 2 + RX['vy'][m] ** 2))
                vals.append(math.sqrt(np.mean(acc)) if acc else np.nan)
            ax[0].plot(tg - A.w0, vals, label=lab)
        if hasattr(A, 'geo'):
            w = A.mask(A.geo['t'])
            ax[1].plot(A.geo['t'][w] - A.w0, A.geo['mr'][w], label=lab)
    ax[0].set_ylabel('velocity rms per second (m/s)')
    ax[1].set_ylabel('mean radius from centroid (m)')
    ax[1].set_xlabel('time since window start (s)')
    for a in ax:
        a.grid(True, alpha=0.3)
        a.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'compare.png'), dpi=110)
    plt.close(fig)
    print(open(path).read())
    print(f'saved {path} and compare.png')


# ----------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('paths', nargs='*', default=['logs'], help='run folder(s), logs dir, or old CSVs')
    ap.add_argument('--compare', action='store_true', help='compare several runs side by side')
    ap.add_argument('--skip', type=float, default=2.0, help='seconds to ignore after flight start (default 2)')
    ap.add_argument('--state-hz', type=float, default=20.0, help='nominal state topic rate (crazyflies.yaml)')
    ap.add_argument('--out', default=None, help='output folder (default: <run>/analysis)')
    a = ap.parse_args()
    global STATE_HZ_NOMINAL
    STATE_HZ_NOMINAL = a.state_hz
    paths = a.paths
    if len(paths) == 1 and not a.compare:
        p = paths[0]
        if os.path.isdir(p) and not glob.glob(os.path.join(p, 'tick_*.csv')):
            sub_ = os.path.join(p, 'runs')
            if os.path.isdir(sub_) or os.path.basename(p.rstrip('/')) == 'runs':
                p = newest_run(p)
        run = Run(p)
        A = Analysis(run, skip=a.skip)
        A.run_all()
        out = a.out or os.path.join(run.dir, 'analysis')
        os.makedirs(out, exist_ok=True)
        figures(A, out)
        s = write_outputs(A, out)
        print(open(s).read())
        print(f'full report: {os.path.join(out, "report.md")}\nfigures   : {out}/fig_*.png')
    elif a.compare or len(paths) > 1:
        if len(paths) > 1 and all(p.endswith('.csv') for p in paths):
            run = Run(paths[0] if len(paths) == 1 else os.path.join(os.path.dirname(paths[0]), 'plain_sph_*.csv'))
            A = Analysis(run)
            A.run_all()
            out = a.out or os.path.join(run.dir, 'analysis')
            os.makedirs(out, exist_ok=True)
            figures(A, out)
            print(open(write_outputs(A, out)).read())
        else:
            compare(paths, a.out or 'compare_out')


if __name__ == '__main__':
    main()
