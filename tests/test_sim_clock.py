#!/usr/bin/env python3
"""Offline self-checks of the simulated-time clock:  python3 tests/test_sim_clock.py

No ROS, no Gazebo: a fake `gz topic -e` (a small python script printing stats messages in
the real text format) stands in for the simulator.
"""
import os
import statistics
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'src', 'drone_controller'))

from drone_controller.sim_clock import GzSimClock, SimTicker, StatsParser   # noqa: E402

FAILS = []


def check(name, ok, detail=''):
    print(('PASS  ' if ok else 'FAIL  ') + name + (f'   {detail}' if detail else ''))
    if not ok:
        FAILS.append(name)


SAMPLE = """sim_time {
  sec: 12
  nsec: 500000000
}
real_time {
  sec: 100
  nsec: 250000000
}
paused: false
iterations: 12500
real_time_factor: 0.12
"""

FAKE = r'''
import sys, time
rtf = float(sys.argv[1]); hz = 10.0; pause_after = float(sys.argv[2]) if len(sys.argv) > 2 else 1e9
t0 = time.time(); sim = 3.0   # does not start at 0, like a real run
last = t0
while True:
    time.sleep(1.0 / hz)
    now = time.time()
    if now - t0 < pause_after:
        sim += (now - last) * rtf
    last = now
    s = int(sim); n = int((sim - s) * 1e9)
    sys.stdout.write("sim_time {\n  sec: %d\n  nsec: %d\n}\nreal_time {\n  sec: %d\n  nsec: 0\n}\n"
                     "paused: false\nreal_time_factor: %.4f\n" % (s, n, int(now - t0), rtf))
    sys.stdout.flush()
'''


def run_clock(rtf, wall_s, pause_after=None):
    fake = os.path.join(tempfile.mkdtemp(), 'fake_gz.py')
    open(fake, 'w').write(FAKE)
    argv = [sys.executable, fake, str(rtf)] + ([str(pause_after)] if pause_after else [])
    clk = GzSimClock(argv=argv)
    tk = SimTicker(clk, 0.05)
    ticks, t_end, last_sim = [], time.monotonic() + wall_s, -1.0
    mono_ok = True
    while time.monotonic() < t_end:
        s = clk.sim_now()
        if s is not None:
            mono_ok &= s >= last_sim
            last_sim = s
        r = tk.poll()
        if r is not None:
            ticks.append(r)
        time.sleep(0.002)
    return clk, tk, ticks, mono_ok


# 1. parser on the real text format
p = StatsParser()
out = None
for ln in SAMPLE.splitlines():
    out = p.feed(ln) or out
check('StatsParser reads sim_time, real_time and rtf', out == (12.5, 100.25, 0.12), str(out))

# 2. RTF 0.5: one tick per 50 ms of SIMULATED time, i.e. one per 100 ms wall
clk, tk, ticks, mono = run_clock(0.5, 5.0)
dts = [d for _, d in ticks]
check('tick count matches simulated time (RTF 0.5, 5 s wall -> ~50 ticks)', 44 <= len(ticks) <= 56, str(len(ticks)))
check('simulated dt per tick is the period (RTF 0.5)',
      bool(dts) and abs(statistics.mean(dts) - 0.05) < 0.003 and max(dts) < 0.075,
      f'mean {statistics.mean(dts) * 1e3:.1f} ms max {max(dts) * 1e3:.1f} ms' if dts else 'no ticks')
check('RTF estimate close to the true 0.5', abs(clk.rtf - 0.5) < 0.05, f'{clk.rtf:.3f}')
check('sim_now never goes backwards', mono)
check('no missed periods when the loop keeps up', tk.missed == 0, str(tk.missed))
clk.close()

# 3. RTF 0.12 (your laptop with 5 drones): same dt per tick, 8x fewer ticks per wall second
clk, tk, ticks, mono = run_clock(0.12, 6.0)
dts = [d for _, d in ticks]
check('tick rate follows RTF (0.12 -> ~2.4 ticks per wall s)', 11 <= len(ticks) <= 17, str(len(ticks)))
check('dt per tick still 50 ms at RTF 0.12',
      bool(dts) and abs(statistics.mean(dts) - 0.05) < 0.004, f'{statistics.mean(dts) * 1e3:.1f} ms' if dts else '')
clk.close()

# 4. simulation pause (gz keeps publishing a frozen sim_time) -> RTF 0, ticks stop, no runaway extrapolation
clk, tk, ticks, mono = run_clock(1.0, 5.0, pause_after=2.0)
last_t = ticks[-1][0] if ticks else 0
check('measured RTF drops to ~0 while the simulation is paused', clk.rtf < 0.05, f'{clk.rtf:.3f}')
check('ticks stop while paused (about 2 s of sim, not 5)', 1.7 <= last_t <= 2.9, f'sim t of last tick {last_t:.2f}')
clk.close()

# 5. a starved loop skips periods instead of bursting ticks
class FakeClock:
    ready = True
    def __init__(self): self.t = 0.0
    def sim_now(self, wall=None): return self.t
fc = FakeClock()
tk = SimTicker(fc, 0.05)
tk.poll()                     # arms at t=0
fc.t = 0.26                   # 5 periods passed
r = tk.poll()
check('late loop: one tick, periods counted as missed', r is not None and tk.missed == 4, f'{r} missed={tk.missed}')
check('no burst after the late tick', tk.poll() is None)

# 6. missing gz -> error recorded, nothing raised
clk = GzSimClock(argv=['/nonexistent/gz'])
time.sleep(0.5)
check('missing command is reported, not raised', (not clk.ready) and clk.error is not None, str(clk.error))
clk.close()

print()
print('ALL OK' if not FAILS else f'{len(FAILS)} FAILED: {FAILS}')
sys.exit(1 if FAILS else 0)
