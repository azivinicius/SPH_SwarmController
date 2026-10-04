#!/usr/bin/env python3
"""Regenerate config/agents.txt and the `robots:` block of config/crazyflies.yaml
for N drones, so the two files can never get out of sync by hand.

Usage: python3 gen_config.py --n 5 --radius 0.5 [--config-dir config]

Drones are spawned evenly on a circle of the given radius around (0, 0)
(a single drone goes at the origin).

IMPORTANT: both files are bind-mounted single files in docker-compose.yaml.
They are rewritten IN PLACE (open(..., 'w') keeps the same inode). Do not
change this to write-temp-then-rename: the container would keep seeing the
old file.
"""
import argparse
import math
import re
from pathlib import Path


def positions(n, radius):
    if n == 1:
        return [(0.0, 0.0)]
    return [(radius * math.cos(2 * math.pi * i / n),
             radius * math.sin(2 * math.pi * i / n)) for i in range(n)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, required=True)
    ap.add_argument('--radius', type=float, default=0.5)
    ap.add_argument('--config-dir', default='config')
    a = ap.parse_args()
    if a.n < 1:
        raise SystemExit('--n must be >= 1')

    pos = positions(a.n, a.radius)
    if a.n > 1:
        min_d = 2 * a.radius * math.sin(math.pi / a.n)
        print(f'[gen_config] {a.n} drones on r={a.radius} m -> neighbour spacing {min_d:.2f} m')
        if min_d < 0.3:
            print('[gen_config] WARNING: spacing < 0.3 m at spawn, raise SPAWN_RADIUS')

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