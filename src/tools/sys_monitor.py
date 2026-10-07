#!/usr/bin/env python3
"""Log CPU, memory, clock speed and temperature once per second, per process group.

Runs INSIDE the container (dev_tmux.sh starts it in the `sys` tmux window) and needs only
the standard library (reads /proc and /sys):

    python3 /root/dev_ws/src/tools/sys_monitor.py --out /root/dev_ws/src/logs/runs/<run_id>/sys.csv

One row per sample. Time is `epoch` (unix s), the same clock as the tick logs, so
analyze_run.py can line it up with RTF and the controller's timing.

Columns
  epoch, ncpu, ncore          logical CPUs (hyper-threads count) and physical cores of the machine
  cores_total                 busy CPUs, whole machine (host-wide: /proc/stat sees outside the container too)
  cores_outside               cores_total minus everything counted inside this container
  load1                       1-minute load average
  mem_used_mb, mem_total_mb   machine memory in use (MemTotal - MemAvailable)
  freq_mhz                    mean current CPU clock (drops when the laptop throttles)
  temp_c                      hottest CPU thermal zone (blank if not readable)
  ctx_per_s                   context switches per second (high = many busy threads fighting for cores)
  throttled                   container CPU-throttle events this interval (blank if no cgroup limit)
  n_proc                      processes inside the container
  cpu_<group>, rss_<group>    busy cores and resident MB of each process group:
      gz        Gazebo (gz sim, its ruby launcher, server/gui)
      cf2       the Crazyflie firmware processes (one per drone)
      server    crazyswarm2's crazyflie_server
      rviz      rviz2
      ctrl      the SPH controller (swarm_sph / plain_sph)
      tools     this monitor, rtf_logger, `gz topic`
      ros       other ROS 2 processes (launch, daemon, ...)
      other     everything else in the container
  n_cf2                       number of cf2 processes (= drones simulated)
"""
import argparse
import os
import signal
import sys
import time

GROUPS = ['gz', 'cf2', 'server', 'rviz', 'ctrl', 'tools', 'ros', 'other']
TCK = os.sysconf('SC_CLK_TCK')
PAGE = os.sysconf('SC_PAGE_SIZE')


def classify(comm, cmd):
    c = cmd
    if comm == 'cf2':
        return 'cf2'
    if 'sys_monitor' in c or 'rtf_logger' in c or 'gz topic' in c:
        return 'tools'
    if 'swarm_sph' in c or 'plain_sph' in c:
        return 'ctrl'
    if 'crazyflie_server' in c:
        return 'server'
    if 'rviz2' in c:
        return 'rviz'
    if (comm in ('ruby', 'gz') or 'gz sim' in c or 'gz-sim' in c or 'gzserver' in c
            or 'ign gazebo' in c or 'gz_' in comm):
        return 'gz'
    if 'ros2' in c or '/opt/ros' in c or '_ros2' in c or 'launch' in c:
        return 'ros'
    return 'other'


def read_stat_cpu():
    """(busy_jiffies, total_jiffies, ncpu, ctxt) host-wide."""
    busy = total = ctxt = 0
    ncpu = 0
    with open('/proc/stat') as f:
        for ln in f:
            if ln.startswith('cpu '):
                v = [int(x) for x in ln.split()[1:]]
                idle = v[3] + (v[4] if len(v) > 4 else 0)
                total = sum(v[:8])
                busy = total - idle
            elif ln.startswith('cpu'):
                ncpu += 1
            elif ln.startswith('ctxt'):
                ctxt = int(ln.split()[1])
    return busy, total, ncpu, ctxt


def read_physical_cores(ncpu):
    """Physical cores from /proc/cpuinfo (sockets x 'cpu cores'); falls back to ncpu."""
    try:
        per_pkg, pkgs = None, set()
        with open('/proc/cpuinfo') as f:
            for ln in f:
                if ln.startswith('cpu cores'):
                    per_pkg = int(ln.split(':')[1])
                elif ln.startswith('physical id'):
                    pkgs.add(ln.split(':')[1].strip())
        if per_pkg:
            return per_pkg * max(len(pkgs), 1)
    except (OSError, ValueError):
        pass
    return ncpu


def read_mem():
    d = {}
    with open('/proc/meminfo') as f:
        for ln in f:
            k, v = ln.split(':', 1)
            d[k] = int(v.split()[0])
    return (d['MemTotal'] - d.get('MemAvailable', d.get('MemFree', 0))) / 1024.0, d['MemTotal'] / 1024.0


def read_freq():
    vals = []
    base = '/sys/devices/system/cpu'
    try:
        for n in os.listdir(base):
            if n.startswith('cpu') and n[3:].isdigit():
                p = f'{base}/{n}/cpufreq/scaling_cur_freq'
                try:
                    with open(p) as f:
                        vals.append(int(f.read()) / 1000.0)
                except OSError:
                    pass
    except OSError:
        pass
    if not vals:
        try:
            with open('/proc/cpuinfo') as f:
                vals = [float(ln.split(':')[1]) for ln in f if ln.startswith('cpu MHz')]
        except OSError:
            pass
    return sum(vals) / len(vals) if vals else None


def read_temp():
    best, anyt = None, None
    base = '/sys/class/thermal'
    try:
        for z in os.listdir(base):
            if not z.startswith('thermal_zone'):
                continue
            try:
                with open(f'{base}/{z}/temp') as f:
                    t = int(f.read()) / 1000.0
                with open(f'{base}/{z}/type') as f:
                    ty = f.read().strip()
            except (OSError, ValueError):
                continue
            anyt = t if anyt is None else max(anyt, t)
            if 'pkg' in ty or 'cpu' in ty.lower() or 'core' in ty.lower():
                best = t if best is None else max(best, t)
    except OSError:
        pass
    return best if best is not None else anyt


def read_throttled():
    for p in ('/sys/fs/cgroup/cpu.stat', '/sys/fs/cgroup/cpu/cpu.stat'):
        try:
            with open(p) as f:
                for ln in f:
                    if ln.startswith('nr_throttled'):
                        return int(ln.split()[1])
        except OSError:
            pass
    return None


def scan_procs(cache):
    """{pid: (group, ticks, rss_mb)}"""
    out = {}
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        try:
            with open(f'/proc/{pid}/stat') as f:
                s = f.read()
            rp = s.rindex(')')
            comm = s[s.index('(') + 1:rp]
            rest = s[rp + 2:].split()
            ticks = int(rest[11]) + int(rest[12])
            if pid not in cache:
                try:
                    with open(f'/proc/{pid}/cmdline', 'rb') as f:
                        cmd = f.read().replace(b'\0', b' ').decode('utf8', 'replace')
                except OSError:
                    cmd = ''
                cache[pid] = classify(comm, cmd)
            try:
                with open(f'/proc/{pid}/statm') as f:
                    rss = int(f.read().split()[1]) * PAGE / 1048576.0
            except (OSError, IndexError, ValueError):
                rss = 0.0
            out[pid] = (cache[pid], ticks, rss)
        except (OSError, ValueError, IndexError):
            continue
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--out', required=True)
    ap.add_argument('--interval', type=float, default=1.0)
    ap.add_argument('--duration', type=float, default=0.0, help='stop after this many seconds (0 = until killed)')
    a = ap.parse_args()
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)

    stop = [False]
    for sg in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sg, lambda *_: stop.__setitem__(0, True))

    cols = (['epoch', 'ncpu', 'ncore', 'cores_total', 'cores_outside', 'load1', 'mem_used_mb', 'mem_total_mb',
             'freq_mhz', 'temp_c', 'ctx_per_s', 'throttled', 'n_proc', 'n_cf2']
            + [f'cpu_{g}' for g in GROUPS] + [f'rss_{g}' for g in GROUPS])
    cache = {}
    prev_busy, prev_total, ncpu, prev_ctxt = read_stat_cpu()
    prev_procs = scan_procs(cache)
    prev_thr = read_throttled()
    t_prev = time.time()
    t_start = t_prev
    print(f'sys_monitor: every {a.interval:g} s -> {a.out}', flush=True)

    def fmt(v):
        return '' if v is None else (f'{v:.3f}' if isinstance(v, float) else str(v))

    with open(a.out, 'w', buffering=1) as f:
        f.write(','.join(cols) + '\n')
        while not stop[0]:
            time.sleep(max(0.05, a.interval - (time.time() - t_prev)))
            now = time.time()
            dt = now - t_prev
            busy, total, ncpu, ctxt = read_stat_cpu()
            dtot = max(total - prev_total, 1)
            cores_total = (busy - prev_busy) / dtot * ncpu
            procs = scan_procs(cache)
            cpu = {g: 0.0 for g in GROUPS}
            rss = {g: 0.0 for g in GROUPS}
            n_cf2 = 0
            for pid, (g, ticks, mb) in procs.items():
                before = prev_procs.get(pid)
                dticks = ticks - (before[1] if before else 0)
                if dticks > 0:
                    cpu[g] += dticks / TCK / dt
                rss[g] += mb
                if g == 'cf2':
                    n_cf2 += 1
            inside = sum(cpu.values())
            mem_used, mem_total = read_mem()
            thr = read_throttled()
            row = {
                'epoch': now, 'ncpu': ncpu, 'ncore': read_physical_cores(ncpu), 'cores_total': cores_total,
                'cores_outside': max(cores_total - inside, 0.0), 'load1': os.getloadavg()[0],
                'mem_used_mb': mem_used, 'mem_total_mb': mem_total, 'freq_mhz': read_freq(),
                'temp_c': read_temp(), 'ctx_per_s': (ctxt - prev_ctxt) / dt,
                'throttled': None if (thr is None or prev_thr is None) else thr - prev_thr,
                'n_proc': len(procs), 'n_cf2': n_cf2,
            }
            for g in GROUPS:
                row[f'cpu_{g}'] = cpu[g]
                row[f'rss_{g}'] = rss[g]
            f.write(','.join(fmt(row[c]) for c in cols) + '\n')
            prev_busy, prev_total, prev_ctxt, prev_procs, prev_thr, t_prev = busy, total, ctxt, procs, thr, now
            if a.duration and now - t_start >= a.duration:
                break
    return 0


if __name__ == '__main__':
    sys.exit(main())