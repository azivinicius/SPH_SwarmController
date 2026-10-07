#!/usr/bin/env python3
"""Summarise sys.csv (from sys_monitor.py) of one or more runs. Standard library only.

    python3 src/tools/sys_report.py src/logs/runs/<run_id>            one run, details
    python3 src/tools/sys_report.py run1 run2 run3 ...                a table, one column per run
    python3 src/tools/sys_report.py --md out.md run1 run2 ...         also write the table to a file

Only the part of the run in which the SPH controller was alive is used (cpu_ctrl > 0.005 busy cores),
so the simulator start-up is not mixed into the numbers. RTF comes from rtf.csv of the same run
(simulated seconds per wall second, same time window).
"""
import argparse
import csv
import os
import sys

GROUPS = ['gz', 'cf2', 'server', 'rviz', 'ctrl', 'tools', 'ros', 'other']
LABEL = {'gz': 'Gazebo', 'cf2': 'firmware (cf2)', 'server': 'crazyflie_server', 'rviz': 'rviz2',
         'ctrl': 'SPH controller', 'tools': 'logging tools', 'ros': 'other ROS', 'other': 'other in container'}


def load(path):
    with open(path, newline='') as f:
        rows = list(csv.DictReader(f))
    out = []
    for r in rows:
        d = {}
        for k, v in r.items():
            try:
                d[k] = float(v)
            except (TypeError, ValueError):
                d[k] = None
        out.append(d)
    return out


def mean(x):
    x = [v for v in x if v is not None]
    return sum(x) / len(x) if x else None


def pct(x, q):
    x = sorted(v for v in x if v is not None)
    if not x:
        return None
    return x[min(len(x) - 1, int(round(q / 100 * (len(x) - 1))))]


def rtf_of(run_dir, t0, t1):
    p = os.path.join(run_dir, 'rtf.csv')
    if not os.path.exists(p):
        return None
    pts = [(r['epoch'], r['sim_time']) for r in load(p) if r.get('epoch') and r.get('sim_time') is not None]
    pts = [q for q in pts if t0 <= q[0] <= t1]
    if len(pts) < 2 or pts[-1][0] - pts[0][0] < 1:
        return None
    return (pts[-1][1] - pts[0][1]) / (pts[-1][0] - pts[0][0])


def summarise(run_dir):
    p = os.path.join(run_dir, 'sys.csv')
    if not os.path.exists(p):
        return None
    rows = load(p)
    live = [r for r in rows if (r.get('cpu_ctrl') or 0) > 0.005]
    scope = 'controller alive'
    if len(live) < 5:
        live, scope = rows, 'whole run (controller never seen)'
    if len(live) < 3:
        return None
    s = {'run': os.path.basename(run_dir.rstrip('/')), 'scope': scope, 'seconds': len(live),
         'ncpu': live[0].get('ncpu'), 'ncore': live[0].get('ncore')}
    s['busy'] = mean([r['cores_total'] for r in live])
    s['busy_p95'] = pct([r['cores_total'] for r in live], 95)
    s['outside'] = mean([r['cores_outside'] for r in live])
    for g in GROUPS:
        s['cpu_' + g] = mean([r.get('cpu_' + g) for r in live])
        s['rss_' + g] = max([r.get('rss_' + g) or 0 for r in live])
    s['mem_max'] = max(r['mem_used_mb'] for r in live)
    s['mem_total'] = live[0]['mem_total_mb']
    s['freq_mean'] = mean([r['freq_mhz'] for r in live])
    s['freq_max'] = max([r['freq_mhz'] for r in rows if r['freq_mhz'] is not None] or [None]) \
        if any(r['freq_mhz'] is not None for r in rows) else None
    temps = [r['temp_c'] for r in live if r['temp_c'] is not None]
    s['temp_max'] = max(temps) if temps else None
    s['ctx'] = mean([r['ctx_per_s'] for r in live])
    s['throttled'] = sum(r['throttled'] or 0 for r in live)
    s['n_cf2'] = max(r['n_cf2'] or 0 for r in live)
    s['rtf'] = rtf_of(run_dir, live[0]['epoch'], live[-1]['epoch'])
    return s


def f(v, nd=2):
    return 'n/a' if v is None else f'{v:.{nd}f}'


def verdict(s):
    ratio = s['busy'] / s['ncore'] if s['ncore'] else None
    if ratio is None:
        return ''
    if ratio >= 1.0:
        return f'CPU FULL: {100 * ratio:.0f} % of the {s["ncore"]:.0f} physical cores'
    if ratio >= 0.7:
        return f'CPU high: {100 * ratio:.0f} % of the {s["ncore"]:.0f} physical cores'
    return f'CPU ok: {100 * ratio:.0f} % of the {s["ncore"]:.0f} physical cores'


def detail(s):
    print(f'run {s["run"]}   ({s["seconds"]} s, {s["scope"]}, {s["n_cf2"]:.0f} drones)')
    print(f'  RTF {f(s["rtf"])}   {verdict(s)}')
    print(f'  busy cores: mean {f(s["busy"])} p95 {f(s["busy_p95"])} of {s["ncpu"]:.0f} logical; outside the container {f(s["outside"])}')
    for g, v in sorted(((g, s['cpu_' + g]) for g in GROUPS), key=lambda kv: -(kv[1] or 0)):
        if (v or 0) >= 0.02:
            print(f'    {LABEL[g]:22s} {f(v)} cores   {s["rss_" + g]:.0f} MB')
    print(f'  memory peak {s["mem_max"]:.0f} of {s["mem_total"]:.0f} MB; clock mean {f(s["freq_mean"], 0)} MHz '
          f'(peak {f(s["freq_max"], 0)}); temp max {f(s["temp_max"], 0)} C; ctx switches {f(s["ctx"], 0)}/s; '
          f'docker throttled {s["throttled"]:.0f}x')


def table(ss):
    rows = [('drones', lambda s: f'{s["n_cf2"]:.0f}'), ('RTF', lambda s: f(s['rtf'])),
            ('cores busy', lambda s: f(s['busy'])), ('busy / physical', lambda s: f(s['busy'] / s['ncore'] if s['ncore'] else None))]
    rows += [(f'cores {LABEL[g]}', (lambda g: lambda s: f(s['cpu_' + g]))(g)) for g in GROUPS]
    rows += [('cores outside', lambda s: f(s['outside'])), ('mem peak MB', lambda s: f(s['mem_max'], 0)),
             ('clock MHz', lambda s: f(s['freq_mean'], 0)), ('temp max C', lambda s: f(s['temp_max'], 0)),
             ('ctx/s', lambda s: f(s['ctx'], 0))]
    lines = ['| metric | ' + ' | '.join(s['run'][-28:] for s in ss) + ' |', '|---|' + '---|' * len(ss)]
    for name, fn in rows:
        lines.append(f'| {name} | ' + ' | '.join(fn(s) for s in ss) + ' |')
    return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--md', default=None)
    a = ap.parse_args()
    ss = []
    for r in a.runs:
        s = summarise(r)
        if s is None:
            print(f'{r}: no usable sys.csv', file=sys.stderr)
        else:
            ss.append(s)
    if not ss:
        return 1
    if len(ss) == 1:
        detail(ss[0])
    else:
        t = table(ss)
        print(t)
        if a.md:
            with open(a.md, 'w') as fh:
                fh.write(t + '\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())