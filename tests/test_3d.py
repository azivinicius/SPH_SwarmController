#!/usr/bin/env python3
"""Offline self-checks of the 3D mode:  python3 tests/test_3d.py

Kernel normalisation and gradient, the sphere potential, the mass/radius solver, and the REAL control
loop (sph_core, dim=3) closed around the particle backend: do N drones settle on the sphere, keep apart,
respect the climb limit and the floor?"""
import math
import os
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path[:0] = [os.path.join(ROOT, 'src', 'drone_controller'), ROOT]

from drone_controller import particle_sim as ps, sph_core            # noqa: E402
from drone_controller.particle3d import Particle3D, kernel3           # noqa: E402
from drone_controller.potential3d import Potential3D                  # noqa: E402
import gen_config as g                                                # noqa: E402

FAILS = []


def check(name, ok, info=''):
    print(('PASS ' if ok else 'FAIL ') + name + (f'  [{info}]' if info else ''))
    if not ok:
        FAILS.append(name)


# --- kernel ------------------------------------------------------------------------------------------
h, n = 0.3, 400
dr = 2 * h / n
tot = sum(kernel3((i + .5) * dr, h) * 4 * math.pi * ((i + .5) * dr) ** 2 * dr for i in range(n))
check('3D kernel integrates to 1', abs(tot - 1) < 1e-6, f'{tot:.8f}')
for st in (1.0, 2.0):
    b = Particle3D('b', 0.2, 0.1, -0.15, h, 1, z_stretch=st)
    e = 1e-6
    ok = True
    gr = Particle3D('a', 0, 0, 0, h, 1, z_stretch=st).grad_W(b)
    for ax, nm in enumerate('xyz'):
        hi, lo = Particle3D('a', 0, 0, 0, h, 1, z_stretch=st), Particle3D('a', 0, 0, 0, h, 1, z_stretch=st)
        setattr(hi, nm, e)
        setattr(lo, nm, -e)
        ok &= abs(gr[ax] - (hi.W(b) - lo.W(b)) / (2 * e)) < 1e-4 * max(1, abs(gr[ax]))
    check(f'grad_W matches finite differences (z_stretch {st})', ok)
a, b = Particle3D('a', 0, 0, 0, h, 1), Particle3D('b', 0, 0, 0, h, 1)
check('coincident particles give a 3-vector gradient', a.grad_W(b) == (0.0, 0.0, 0.0))

# --- potential ----------------------------------------------------------------------------------------
P = Potential3D(1.0, 2.0, 3.0, 0.5)
f_out = P.force(1.0 + 0.8, 2.0, 3.0, 1.0, 2.0)
f_in = P.force(1.0 + 0.2, 2.0, 3.0, 1.0, 2.0)
check('outside: pulled in, constant magnitude k', f_out[0] < 0 and abs(np.linalg.norm(f_out) - 2.0) < 1e-9)
check('inside: pushed out', f_in[0] > 0)
check('on the surface / centre: zero 3-vector, no crash', len(P.force(1.5, 2, 3)) == 3 and not P.force(1, 2, 3).any())

# --- solver -------------------------------------------------------------------------------------------
R5 = g.sphere_radius_for_spacing(5, 0.3)
pts = g.fibonacci_sphere(5, R5)
check('sphere radius gives the wanted mean spacing', abs(g.mean_nn(pts) - 0.3) < 1e-9)
m = g.solve_mass3d(pts, 0.3, 1000.0)
dens = [sum(m * kernel3(math.dist(a_, b_), 0.3) for b_ in pts) for a_ in pts]
check('solved mass gives mean density rho0', abs(sum(dens) / 5 - 1000.0) < 1e-6)


# --- closed loop --------------------------------------------------------------------------------------
def run(n=8, gain=0.05, f_k=1.0, T=60.0, drift=0.0, max_vz=0.25, z_min=0.25, zc=1.0, stretch=1.0, jitter=0.12):
    Rr = g.sphere_radius_for_spacing(n, 0.3)
    mass = g.solve_mass3d(g.fibonacci_sphere(n, Rr), 0.3, 1000.0)
    cfg = dict(sph_core.COMMON_DEFAULTS)
    cfg.update(particle_mass=mass, smoothing_h=0.3, gamma=2.0, zetta_extra=10.0, xc=0.0, yc=0.0, zc=zc, R=Rr,
               f_k=f_k, sph_gain=gain, dim=3.0, z_stretch=stretch, max_vz=max_vz, z_min=z_min, total_robots=n,
               run_id='t3d', run_label='t3d', git_rev='x', log_dir=tempfile.mkdtemp())
    names = [f'cf_{i}' for i in range(n)]
    be = ps.ParticleBackend(names, g.grid_positions(n, 0.3), 0.5, 0.15, 0.1, 0.0, tau_z=0.3,
                            z_drift=drift, z_jitter=jitter)
    t = [0.0]
    core, _ = sph_core.build_core(cfg, lambda nn_: list(nn_), density_on_rx=False, tag='swarm',
                                  epoch_fn=lambda: 1e9 + t[0])
    vzmax, zlo = 0.0, 9.0
    for _ in range(int(T / 0.05)):
        t[0] += 0.05
        be.step(0.05)
        for i, nm in enumerate(names):
            core.on_state(nm, t[0], be.state(i))
        for nm, c in core.step(t[0]).items():
            assert len(c) == 3, 'dim=3 must publish (vx, vy, vz)'
            vzmax = max(vzmax, abs(c[2]))
            be.command(nm, *c)
        zlo = min(zlo, be.z.min())
    Pn = np.vstack([be.x, be.y, be.z]).T
    d = np.linalg.norm(Pn - [0, 0, zc], axis=1)
    nn_ = [min(np.linalg.norm(Pn[i] - Pn[j]) for j in range(n) if j != i) for i in range(n)]
    return dict(R=Rr, rerr=float(abs(d - Rr).max()), dmin=float(min(nn_)), z=(float(be.z.min()), float(be.z.max())),
                vzmax=vzmax, zlo=float(zlo), core=core)


r = run()
check('8 drones settle on the sphere', r['rerr'] < 0.08, f'max |r-R| {r["rerr"]:.3f} (R {r["R"]:.3f})')
check('drones keep apart (recommended gain 0.05, f_k 1: closest > 0.22 m)', r['dmin'] > 0.22, f'closest {r["dmin"]:.3f} m')
check('formation is really 3D (spread in z)', r['z'][1] - r['z'][0] > 0.2, f'z {r["z"][0]:.2f}..{r["z"][1]:.2f}')
check('climb command respects max_vz', r['vzmax'] <= 0.25 + 1e-9, f'{r["vzmax"]:.3f}')
r2 = run(max_vz=0.08, T=40.0)
check('a lower max_vz is obeyed', r2['vzmax'] <= 0.08 + 1e-9, f'{r2["vzmax"]:.3f}')
r3 = run(zc=0.5, z_min=0.4, T=60.0)
check('floor: command never pushes below z_min (sphere half under the floor)', r3['zlo'] > 0.30, f'lowest z {r3["zlo"]:.2f}')
r4 = run(drift=-0.02, f_k=2.0)
check('small sinking drift (-0.02 m/s) is held by the loop (f_k 2)', r4['rerr'] < 0.1, f'max |r-R| {r4["rerr"]:.3f}')
r5 = run(stretch=2.0, n=5)
check('z_stretch runs and settles', r5['rerr'] < 0.1 and r5['dmin'] > 0.2, f'rerr {r5["rerr"]:.3f} dmin {r5["dmin"]:.3f}')

# --- the 3D tick log carries the z columns ---------------------------------------------------------------
import csv                                        # noqa: E402
run_dir = os.path.join(r['core'].log.dir if hasattr(r['core'].log, 'dir') else '', '')
logdir = r['core'].log.run_dir if hasattr(r['core'].log, 'run_dir') else None
if logdir and os.path.isdir(logdir):
    f = [x for x in os.listdir(logdir) if x.startswith('tick_cf_0')]
    if f:
        row = next(csv.DictReader(open(os.path.join(logdir, f[0]))))
        check('tick log has cmd_z / used_z', 'cmd_z' in row and 'used_z' in row)

print('\nALL PASS' if not FAILS else '\nFAILED: ' + ', '.join(FAILS))
sys.exit(1 if FAILS else 0)