#!/usr/bin/env python3
"""Regenerate config/agents.txt and the `robots:` block of config/crazyflies.yaml
for N drones, so the two files can never get out of sync by hand.

Usage: python3 gen_config.py --n 5 [--radius 0.5 | --radius auto] [--config-dir config]

Drones are spawned evenly on a circle of the given radius around (0, 0)
(a single drone goes at the origin).

--radius auto (the default) picks the ring radius for N drones so that the SPH
density on the ring equals rho_0 for the SAME particle mass, whatever N is:
density grows with the number of drones inside the kernel, so the ring has to
grow with N. It uses --mass / --h / --rho0 (keep them equal to SPH_MASS /
SPH_H / SPH_RHO0 in swarm.conf) and never goes below --min-spacing between
neighbouring drones.  --print-radius just prints the radius and exits.

IMPORTANT: both files are bind-mounted single files in docker-compose.yaml.
They are rewritten IN PLACE (open(..., 'w') keeps the same inode). Do not
change this to write-temp-then-rename: the container would keep seeing the
old file.
"""
import argparse
import math
import re
import sys
from pathlib import Path


def kernel(r, h):
    """Cubic spline kernel -- identical to W() in particle.py."""
    k = r / h
    c = 10.0 / (7.0 * math.pi * h * h)
    if k <= 1.0:
        return c * (1 - 1.5 * k ** 2 + 0.75 * k ** 3)
    if k <= 2.0:
        return c * 0.25 * (2 - k) ** 3
    return 0.0


def ring_density(n, radius, mass, h):
    """SPH density at one drone of an evenly spaced ring (self term included)."""
    return mass * sum(kernel(2 * radius * math.sin(math.pi * j / n), h) for j in range(n))


def auto_radius(n, mass, h, rho_target, min_spacing):
    """Smallest ring radius whose density equals rho_target (density falls as
    the ring grows), but never below min_spacing between neighbours."""
    if n == 1:
        return 0.5  # radius is irrelevant for a single drone; keep the old default
    r_min = min_spacing / (2 * math.sin(math.pi / n))
    r_max = 100.0
    if ring_density(n, r_max, mass, h) > rho_target:
        raise SystemExit(
            f'[gen_config] mass {mass} is too large: even a lone drone has density '
            f'{mass * kernel(0.0, h):.0f} > target {rho_target:.0f}. Lower SPH_MASS.')
    if ring_density(n, r_min, mass, h) <= rho_target:
        print(f'[gen_config] WARNING: at the minimum spacing ({min_spacing} m) the density is only '
              f'{ring_density(n, r_min, mass, h):.0f} < target {rho_target:.0f}; using the minimum '
              f'radius (SPH_MASS is low for {n} drones)', file=sys.stderr)
        return r_min
    lo, hi = r_min, r_max
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if ring_density(n, mid, mass, h) > rho_target:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def spacing_radius(n, spacing):
    """Ring radius whose neighbour spacing is `spacing`."""
    return spacing / (2 * math.sin(math.pi / n)) if n > 1 else 0.5


def solve_mass(n, spacing, h, rho0):
    """SPH mass that gives density rho0 on a ring with the wanted neighbour spacing."""
    per_unit_mass = ring_density(n, spacing_radius(n, spacing), 1.0, h)
    return rho0 / per_unit_mass


def positions(n, radius):
    if n == 1:
        return [(0.0, 0.0)]
    return [(radius * math.cos(2 * math.pi * i / n),
             radius * math.sin(2 * math.pi * i / n)) for i in range(n)]


def fibonacci_sphere(n, radius, centre=(0.0, 0.0, 0.0)):
    """n roughly evenly spaced points ON a sphere (golden-angle spiral)."""
    if n == 1:
        return [(centre[0], centre[1], centre[2] + radius)]
    ga = math.pi * (3.0 - math.sqrt(5.0))
    pts = []
    for i in range(n):
        zz = 1.0 - 2.0 * (i + 0.5) / n
        rr = math.sqrt(max(0.0, 1.0 - zz * zz))
        pts.append((centre[0] + radius * rr * math.cos(ga * i), centre[1] + radius * rr * math.sin(ga * i),
                    centre[2] + radius * zz))
    return pts


def mean_nn(points):
    return sum(min(math.dist(a, b) for b in points if b is not a) for a in points) / len(points)


def sphere_radius_for_spacing(n, spacing):
    """Radius of the sphere on which n evenly spread points have mean nearest-neighbour distance `spacing`."""
    return spacing / mean_nn(fibonacci_sphere(n, 1.0))


def kernel3(r, h):
    k = r / h
    w = (1 - 1.5 * k * k + 0.75 * k ** 3) if k <= 1 else (0.25 * (2 - k) ** 3 if k <= 2 else 0.0)
    return w / (math.pi * h ** 3)


def solve_mass3d(points, h, rho0):
    """Particle mass for which the 3D SPH density of these points averages rho0 (self term included)."""
    dens = [sum(kernel3(math.dist(a, b), h) for b in points) for a in points]
    return rho0 / (sum(dens) / len(dens))


def grid_positions(n, spacing):
    """Square-ish grid centred on (0,0), `spacing` m between neighbours; an incomplete last row is centred."""
    if n == 1:
        return [(0.0, 0.0)]
    cols = math.ceil(math.sqrt(n))
    rows = math.ceil(n / cols)
    pts = []
    for r in range(rows):
        k = min(cols, n - r * cols)
        for c in range(k):
            pts.append(((c - (k - 1) / 2) * spacing, ((rows - 1) / 2 - r) * spacing))
    return pts


COLORS = [(0.9, 0.2, 0.2), (0.2, 0.6, 0.95), (0.3, 0.8, 0.3), (0.95, 0.75, 0.15), (0.7, 0.35, 0.85),
          (0.2, 0.85, 0.85), (0.95, 0.5, 0.15), (0.9, 0.9, 0.9)]


def world_sdf(pos, z, radius, name='particles', sphere_r=0.05, potential_sphere=None):
    """SDF world for the physics-less particle run: zero gravity, models cf_i that carry only a coloured
    sphere (no collision), moved by /world/<name>/set_pose_vector.  Models are NOT static, because the
    Physics system is what applies the pose commands; with no gravity and no collision they just sit
    where they were put."""
    ground = max(6.0, 4.0 * radius + 2.0)
    models = ''
    for i, (x, y) in enumerate(pos):
        r, g, b = COLORS[i % len(COLORS)]
        models += f"""
    <model name="cf_{i}">
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
      <link name="body">
        <gravity>false</gravity>
        <inertial><mass>0.03</mass>
          <inertia><ixx>1e-5</ixx><iyy>1e-5</iyy><izz>1e-5</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <visual name="v">
          <geometry><sphere><radius>{sphere_r}</radius></sphere></geometry>
          <material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>
        </visual>
      </link>
    </model>"""
    if potential_sphere:
        px, py, pz, pr = potential_sphere
        models += f"""
    <model name="potential_sphere">
      <static>true</static>
      <pose>{px:.4f} {py:.4f} {pz:.4f} 0 0 0</pose>
      <link name="s"><visual name="v">
        <transparency>0.8</transparency>
        <geometry><sphere><radius>{pr:.4f}</radius></sphere></geometry>
        <material><ambient>0.9 0.9 0.2 1</ambient><diffuse>0.9 0.9 0.2 1</diffuse></material>
      </visual></link>
    </model>"""
    return f"""<?xml version="1.0" ?>
<!-- generated by gen_config.py (world option): particles without physics (zero gravity, no collisions) -->
<sdf version="1.8">
  <world name="{name}">
    <physics name="p" type="dart"><max_step_size>0.02</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <gravity>0 0 0</gravity>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name="ground">
      <static>true</static>
      <link name="g"><visual name="v">
        <geometry><plane><normal>0 0 1</normal><size>{ground:.1f} {ground:.1f}</size></plane></geometry>
        <material><ambient>0.35 0.37 0.4 1</ambient><diffuse>0.35 0.37 0.4 1</diffuse></material>
      </visual></link>
    </model>{models}
  </world>
</sdf>
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, required=True)
    ap.add_argument('--radius', default='auto', help="ring radius in metres, or 'auto'")
    ap.add_argument('--mass', default='7000', help="SPH_MASS (for --radius auto), or 'auto' = solve it from --spacing")
    ap.add_argument('--spacing', type=float, default=0.3, help="wanted neighbour spacing, m (used with --mass auto)")
    ap.add_argument('--print-mass', action='store_true', help='print the mass and exit (with --mass auto)')
    ap.add_argument('--h', type=float, default=3.0, help='SPH_H (for --radius auto)')
    ap.add_argument('--rho0', type=float, default=1000.0, help='SPH_RHO0 = target density (for --radius auto)')
    ap.add_argument('--min-spacing', type=float, default=0.3, help='smallest allowed neighbour spacing, m')
    ap.add_argument('--print-radius', action='store_true', help='print the radius and exit')
    ap.add_argument('--config-dir', default='config')
    ap.add_argument('--layout', choices=['grid', 'ring'], default='ring',
                    help="spawn layout: 'grid' (spacing --spacing) or 'ring' (radius --radius)")
    ap.add_argument('--dim', type=int, choices=[2, 3], default=2,
                    help="3: --mass auto / --radius auto solve for a SPHERE (Fibonacci points, 3D kernel)")
    ap.add_argument('--sphere', default='', help="draw the potential sphere in --world: 'xc,yc,zc,R'")
    ap.add_argument('--world', default='', help='also write the particle-run SDF world to this path')
    ap.add_argument('--world-only', action='store_true', help='with --world: do not touch agents.txt / crazyflies.yaml')
    ap.add_argument('--fixed-z', type=float, default=0.5, help='height of the particles in --world')
    a = ap.parse_args()
    if a.n < 1:
        raise SystemExit('--n must be >= 1')

    if a.dim == 3 and (str(a.mass).lower() == 'auto' or a.radius == 'auto'):
        if a.n < 2:
            raise SystemExit('--dim 3 auto needs --n >= 2')
        R3 = sphere_radius_for_spacing(a.n, a.spacing) if a.radius == 'auto' else float(a.radius)
        pts3 = fibonacci_sphere(a.n, R3)
        m3 = solve_mass3d(pts3, a.h, a.rho0)
        print(f'[gen_config] 3D: {a.n} drones on a sphere r={R3:.3f} m (mean spacing {mean_nn(pts3):.3f} m), '
              f'h {a.h:g}, rho0 {a.rho0:g} -> mass {m3:.4g}', file=sys.stderr)
        if a.print_mass:
            print(round(m3, 4))
            return
        if a.print_radius:
            print(round(R3, 4))
            return
        if str(a.mass).lower() == 'auto':
            a.mass = round(m3, 4)
        if a.radius == 'auto':
            a.radius = round(R3, 4)

    mass_auto = str(a.mass).lower() == 'auto' and a.dim == 2
    if mass_auto:
        if a.n < 2:
            raise SystemExit('--mass auto needs --n >= 2')
        a.mass = round(solve_mass(a.n, a.spacing, a.h, a.rho0), 3)
        print(f'[gen_config] mass for {a.n} drones at {a.spacing:g} m spacing (h {a.h:g}, rho0 {a.rho0:g}): {a.mass:g}',
              file=sys.stderr)
        if a.print_mass:
            print(a.mass)
            return
        if a.radius == 'auto':
            a.radius = round(spacing_radius(a.n, a.spacing), 3)
    else:
        a.mass = float(a.mass)
    if a.print_mass:
        raise SystemExit('--print-mass needs --mass auto')
    if a.radius == 'auto':
        a.radius = round(auto_radius(a.n, a.mass, a.h, a.rho0, a.min_spacing), 3)
        print(f'[gen_config] auto radius for {a.n} drones: {a.radius} m '
              f'(density {ring_density(a.n, a.radius, a.mass, a.h):.0f}, '
              f'mass {a.mass:g}, h {a.h:g})', file=sys.stderr)
    else:
        a.radius = float(a.radius)
    if a.print_radius:
        print(a.radius)
        return

    if a.layout == 'grid':
        pos = grid_positions(a.n, a.spacing)
        min_d = a.spacing if a.n > 1 else float('inf')
        print(f'[gen_config] {a.n} drones on a grid, spacing {a.spacing:g} m')
    else:
        pos = positions(a.n, a.radius)
        min_d = 2 * a.radius * math.sin(math.pi / a.n) if a.n > 1 else float('inf')
        if a.n > 1:
            print(f'[gen_config] {a.n} drones on r={a.radius} m -> neighbour spacing {min_d:.2f} m')
    if a.n > 1 and min_d < 0.3 - 1e-3:
        print('[gen_config] WARNING: spacing < 0.3 m at spawn, raise SPAWN_RADIUS / --spacing')

    if a.world:
        sph = tuple(float(v) for v in a.sphere.split(',')) if a.sphere else None
        Path(a.world).write_text(world_sdf(pos, a.fixed_z, max(math.hypot(x, y) for x, y in pos), potential_sphere=sph))
        print(f'[gen_config] wrote world {a.world} ({a.n} particles at z={a.fixed_z:g})')
        if a.world_only:
            return

    cfg = Path(a.config_dir)

    # agents.txt  ("x, y" per line, same order as the yaml)
    with open(cfg / 'agents.txt', 'w') as f:
        f.write('\n'.join(f'{x:.4f}, {y:.4f}' for x, y in pos))

    # crazyflies.yaml: replace only the robots: block
    yaml_path = cfg / 'crazyflies.yaml'
    text = yaml_path.read_text()
    block = 'robots:\n' + ''.join(
        f'  cf_{i}:\n'
        f'    enabled: true\n'
        f'    uri: udp://127.0.0.1:{19850 + i}\n'
        f'    initial_position: [{x:.4f}, {y:.4f}, 0.0]\n'
        f'    type: cf_sim\n'
        for i, (x, y) in enumerate(pos))
    new, count = re.subn(r'^robots:\n.*?(?=^robot_types:)', block + '\n', text,
                         flags=re.S | re.M)
    if count != 1:
        raise SystemExit('could not find the robots: ... robot_types: block in crazyflies.yaml')
    with open(yaml_path, 'w') as f:
        f.write(new)
    print(f'[gen_config] wrote {cfg}/agents.txt and {cfg}/crazyflies.yaml ({a.n} drones)')


if __name__ == '__main__':
    main()