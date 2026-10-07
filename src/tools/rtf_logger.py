#!/usr/bin/env python3
"""Log Gazebo's real-time factor (RTF) to a CSV, with wall-clock timestamps.

Runs INSIDE the container (dev_tmux.sh starts it in the `rtf` tmux window):

    python3 /root/dev_ws/src/tools/rtf_logger.py --out /root/dev_ws/src/logs/runs/<run_id>/rtf.csv

Columns: epoch (unix s, same clock as the tick logs), sim_time (s), real_time (s), rtf.
RTF = simulated seconds per wall second. 1.0 = real time; your logs showed ~0.4.
analyze_run.py reads it to convert wall-time numbers (oscillation frequency, lag) into
simulated time and to see whether RTF wobbles during the run.

It parses the text dump of `gz topic -e -t <world>/stats` (same format as rtf_log.txt).
"""
import argparse
import os
import re
import subprocess
import sys
import time


def find_topic(timeout):
    t_end = time.time() + timeout
    while time.time() < t_end:
        try:
            out = subprocess.run(['gz', 'topic', '-l'], capture_output=True, text=True,
                                 timeout=10).stdout.split()
        except (OSError, subprocess.TimeoutExpired):
            out = []
        cands = [t for t in out if t.endswith('/stats')]
        if cands:
            # prefer the world stats topic
            cands.sort(key=lambda t: ('/world/' not in t, len(t)))
            return cands[0]
        time.sleep(2.0)
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--out', required=True)
    ap.add_argument('--topic', default=None, help='default: auto-detect */stats')
    ap.add_argument('--wait', type=float, default=180.0, help='seconds to wait for the simulator')
    ap.add_argument('--min-interval', type=float, default=0.25, help='min seconds between rows')
    a = ap.parse_args()

    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    topic = a.topic or find_topic(a.wait)
    if not topic:
        print('rtf_logger: no */stats topic found (is Gazebo running?)', file=sys.stderr)
        return 1
    print(f'rtf_logger: {topic} -> {a.out}', flush=True)
    proc = subprocess.Popen(['gz', 'topic', '-e', '-t', topic], stdout=subprocess.PIPE,
                            text=True, bufsize=1)
    cur, vals = None, {'sim': [0, 0], 'real': [0, 0]}
    last = 0.0
    with open(a.out, 'w', buffering=1) as f:
        f.write('epoch,sim_time,real_time,rtf\n')
        try:
            for line in proc.stdout:
                line = line.strip()
                if line.startswith('sim_time'):
                    cur = 'sim'
                elif line.startswith('real_time {'):
                    cur = 'real'
                elif line.startswith('}'):
                    cur = None
                elif cur and line.startswith('sec:'):
                    vals[cur][0] = int(line.split(':')[1])
                elif cur and line.startswith('nsec:'):
                    vals[cur][1] = int(line.split(':')[1])
                elif line.startswith('real_time_factor:'):
                    now = time.time()
                    if now - last >= a.min_interval:
                        last = now
                        sim = vals['sim'][0] + vals['sim'][1] / 1e9
                        real = vals['real'][0] + vals['real'][1] / 1e9
                        f.write(f'{now:.3f},{sim:.4f},{real:.4f},{float(line.split(":")[1]):.5f}\n')
                    vals = {'sim': [0, 0], 'real': [0, 0]}
        except KeyboardInterrupt:
            pass
        finally:
            proc.terminate()
    return 0


if __name__ == '__main__':
    sys.exit(main())
