"""Plot density, pressure, external force, and trajectories from plain_sph's logs.

Run manually after a test, not part of the sim:
    python3 plot_plain_sph.py          # every plain_sph_*.csv in logs/
    python3 plot_plain_sph.py -n 3     # only cf_0..cf_2 (ignore stale CSVs from bigger runs)

Produces two images in logs/ and opens both:
    plain_sph_metrics.png   -- rho, P, |F_ext|, |v| vs time (unchanged)
    plain_sph_potential.png -- potential field + target circle + drone paths
"""
import argparse
import csv
import glob
import math
import os
import platform
import re
import subprocess

import numpy as np
import matplotlib
matplotlib.use('Agg')  # no display needed for rendering -- saves PNGs directly
import matplotlib.pyplot as plt

LOG_DIR = os.path.join(os.path.dirname(__file__), 'logs')

# Potential parameters -- keep these in sync with the node's xc / yc / R parameters
XC = 2.0
YC = 2.0
R = 0.5


def load(path):
    t, rho, P, fx, fy, vx, vy, x, y = [], [], [], [], [], [], [], [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            t.append(float(row['t']))
            rho.append(float(row['rho']))
            P.append(float(row['pressure']))
            fx.append(float(row['ext_force_x']))
            fy.append(float(row['ext_force_y']))
            vx.append(float(row['vel_x']))
            vy.append(float(row['vel_y']))
            # Older logs have no position columns -- skip position for those
            if row.get('x') not in (None, '') and row.get('y') not in (None, ''):
                x.append(float(row['x']))
                y.append(float(row['y']))
    return t, rho, P, fx, fy, vx, vy, x, y


def phi(x, y):
    """Same potential as drone_controller/potential.py: phi = s^2,
    s = ((x-xc)^2 + (y-yc)^2)/R^2 - 1"""
    s = ((x - XC) ** 2 + (y - YC) ** 2) / R ** 2 - 1.0
    return s * s


def open_in_viewer(path):
    """Best-effort: open path in whatever the OS's default image viewer is.
    Never raises -- if this fails, the file is still saved, just not opened."""
    try:
        system = platform.system()
        if system == 'Linux':
            subprocess.Popen(['xdg-open', path],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        elif system == 'Darwin':
            subprocess.Popen(['open', path])
        elif system == 'Windows':
            os.startfile(path)  # noqa: only exists on Windows
        else:
            print(f'(unrecognized OS "{system}" -- open manually: {path})')
    except Exception as exc:
        print(f'(could not auto-open the image: {exc} -- open manually: {path})')


parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
parser.add_argument('-n', '--drones', type=int, default=None,
                    help='only plot cf_0 .. cf_{N-1}; ignores stale CSVs from earlier runs '
                         'with more drones (default: plot every plain_sph_*.csv found)')
args = parser.parse_args()

files = sorted(glob.glob(os.path.join(LOG_DIR, 'plain_sph_*.csv')))
if args.drones is not None:
    def _keep(path):
        m = re.search(r'plain_sph_cf_(\d+)\.csv$', os.path.basename(path))
        return m is not None and int(m.group(1)) < args.drones
    skipped = [os.path.basename(f) for f in files if not _keep(f)]
    files = [f for f in files if _keep(f)]
    if skipped:
        print(f'Ignoring (id >= {args.drones}): {", ".join(skipped)}')
if not files:
    print(f'No matching plain_sph_*.csv files found in {LOG_DIR}')
    raise SystemExit(1)

data = {}
for path in files:
    name = os.path.basename(path).replace('plain_sph_', '').replace('.csv', '')
    data[name] = load(path)

# ---------------------------------------------------------------- figure 1 (existing)
fig, axes = plt.subplots(4, 1, figsize=(9, 13), sharex=True)

for name, (t, rho, P, fx, fy, vx, vy, x, y) in data.items():
    ext_mag = [(a**2 + b**2) ** 0.5 for a, b in zip(fx, fy)]
    speed = [(a**2 + b**2) ** 0.5 for a, b in zip(vx, vy)]

    axes[0].plot(t, rho, label=name)
    axes[1].plot(t, P, label=name)
    axes[2].plot(t, ext_mag, label=name)
    axes[3].plot(t, speed, label=name)

axes[0].set_ylabel('density (rho)')
axes[1].set_ylabel('pressure (P)')
axes[2].set_ylabel('|external force|')
axes[3].set_ylabel('|velocity|')
axes[3].set_xlabel('time (s)')
for ax in axes:
    ax.legend()
    ax.grid(True, alpha=0.3)

fig.tight_layout()
metrics_path = os.path.join(LOG_DIR, 'plain_sph_metrics.png')
fig.savefig(metrics_path, dpi=120)
plt.close(fig)
print(f'Saved: {metrics_path}')

# ---------------------------------------------------------------- figure 2 (new)
have_pos = {n: d for n, d in data.items() if d[7]}
potential_path = None

if not have_pos:
    print('No x/y columns in the logs -- rerun the sim with the updated plain_sph.py '
          'to get the potential/position plot.')
else:
    fig2, (ax_map, ax_r) = plt.subplots(
        1, 2, figsize=(15, 6.5), gridspec_kw={'width_ratios': [1.1, 1]})

    # Plot window: covers the circle and every trajectory, with a margin
    all_x = [v for d in have_pos.values() for v in d[7]] + [XC - R, XC + R]
    all_y = [v for d in have_pos.values() for v in d[8]] + [YC - R, YC + R]
    margin = 0.3 * max(max(all_x) - min(all_x), max(all_y) - min(all_y), 2 * R)
    xmin, xmax = min(all_x) - margin, max(all_x) + margin
    ymin, ymax = min(all_y) - margin, max(all_y) + margin

    # Potential field background (log scale so the valley at the circle is visible)
    gx, gy = np.meshgrid(np.linspace(xmin, xmax, 300), np.linspace(ymin, ymax, 300))
    field = np.log10(phi(gx, gy) + 1e-3)
    cf = ax_map.contourf(gx, gy, field, levels=40, cmap='viridis', alpha=0.85)
    fig2.colorbar(cf, ax=ax_map, label='log10(phi + 1e-3)')

    # Target curve: phi = 0 is the circle of radius R around (xc, yc)
    th = np.linspace(0, 2 * math.pi, 400)
    ax_map.plot(XC + R * np.cos(th), YC + R * np.sin(th),
                'w--', lw=2, label=f'target circle (R={R})')
    ax_map.plot(XC, YC, 'w+', ms=12, mew=2)

    colors = plt.rcParams['axes.prop_cycle'].by_key()['color']
    for i, (name, (t, *_rest, x, y)) in enumerate(have_pos.items()):
        c = colors[i % len(colors)]
        ax_map.plot(x, y, '-', color=c, lw=1.8, label=name)
        ax_map.plot(x[0], y[0], 'o', color=c, mec='k', ms=8)   # start
        ax_map.plot(x[-1], y[-1], 's', color=c, mec='k', ms=8)  # end

        # Distance to circle centre vs time (only rows that have position)
        t_pos = t[:len(x)]
        r = [math.hypot(a - XC, b - YC) for a, b in zip(x, y)]
        ax_r.plot(t_pos, r, color=c, label=name)

    ax_map.set_xlim(xmin, xmax)
    ax_map.set_ylim(ymin, ymax)
    ax_map.set_aspect('equal')
    ax_map.set_xlabel('x (m)')
    ax_map.set_ylabel('y (m)')
    ax_map.set_title('Potential field and drone paths  (o = start, ■ = end)')
    ax_map.legend(loc='upper right', fontsize=8)

    ax_r.axhline(R, color='k', ls='--', label='R (target)')
    ax_r.set_xlabel('time (s)')
    ax_r.set_ylabel('distance to (xc, yc) (m)')
    ax_r.set_title('Radial distance vs time')
    ax_r.grid(True, alpha=0.3)
    ax_r.legend()

    fig2.tight_layout()
    potential_path = os.path.join(LOG_DIR, 'plain_sph_potential.png')
    fig2.savefig(potential_path, dpi=120)
    plt.close(fig2)
    print(f'Saved: {potential_path}')

print('Opening in default image viewer...')
open_in_viewer(metrics_path)
if potential_path:
    open_in_viewer(potential_path)  