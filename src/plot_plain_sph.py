"""Plot density, pressure, and external force from plain_sph's logs.

Run manually after a test, not part of the sim:
    python3 plot_plain_sph.py
"""
import csv
import glob
import os
import platform
import subprocess

import matplotlib
matplotlib.use('Agg')  # no display needed for rendering -- saves PNGs directly
import matplotlib.pyplot as plt

LOG_DIR = os.path.join(os.path.dirname(__file__), 'logs')

def load(path):
    t, rho, P, fx, fy, vx, vy = [], [], [], [], [], [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            t.append(float(row['t']))
            rho.append(float(row['rho']))
            P.append(float(row['pressure']))
            fx.append(float(row['ext_force_x']))
            fy.append(float(row['ext_force_y']))
            vx.append(float(row['vel_x']))
            vy.append(float(row['vel_y']))
    return t, rho, P, fx, fy, vx, vy

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

files = sorted(glob.glob(os.path.join(LOG_DIR, 'plain_sph_*.csv')))
if not files:
    print(f'No plain_sph_*.csv files found in {LOG_DIR}')
    raise SystemExit(1)

fig, axes = plt.subplots(4, 1, figsize=(9, 13), sharex=True)

for path in files:
    name = os.path.basename(path).replace('plain_sph_', '').replace('.csv', '')
    t, rho, P, fx, fy, vx, vy = load(path)
    ext_mag = [(x**2 + y**2) ** 0.5 for x, y in zip(fx, fy)]
    speed = [(x**2 + y**2) ** 0.5 for x, y in zip(vx, vy)]

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
out_path = os.path.join(LOG_DIR, 'plain_sph_metrics.png')
fig.savefig(out_path, dpi=120)
print(f'Saved: {out_path}')

open_in_viewer(out_path)
print('Opening in default image viewer...')