# SPH_Project — CrazySim + Crazyswarm2, custom multi-drone controller

A from-scratch ROS2 package (`drone_controller`) built and tested step-by-step
against CrazySim (Gazebo) + Crazyswarm2, running entirely inside one Docker
container. The project now has two tracks that share the same underlying SPH
physics: an **offline, pure-Python tuning loop** (CMA-ES over the SPH
parameters, no Gazebo needed) and the **in-the-loop ROS2 controller** that
actually flies the simulated drones with those tuned parameters. This doc is
the running reference for how to build it, run it, and the real gotchas
already hit along the way — update it as things change.

## Hardware / OS notes

- Native Ubuntu 24.04 (not WSL2, not macOS).
- Dual-GPU laptop: Intel UHD 620 (integrated) + NVIDIA GeForce MX110 (discrete).
  **Intel is the active rendering path, deliberately** — it was already working,
  is more than adequate for Gazebo Garden, and this machine's NVIDIA driver
  stack has been independently flaky. `docker-compose.yaml` reflects this:
  `gpus: all` stays commented out; the `/dev/dri` device passthrough is what's
  actually used.

## Project structure

```
SPH_Project/
├── docker-compose.yaml
├── docker/
│   └── sim.Dockerfile
├── config/
│   ├── crazyflies.yaml            # robot list + per-robot logging config
│   ├── agents.txt                 # matching Gazebo spawn positions
│   └── sitl_multiagent_headless.sh  # headless Gazebo launch (default -- see Known issue #7)
└── src/
    ├── dev_setup.bash             # workaround script — see Known issue #1
    ├── plot_plain_sph.py          # offline: turns plain_sph_*.csv logs into a PNG
    ├── logs/                      # CSV telemetry + generated plots land here (bind-mounted)
    └── drone_controller/
        ├── package.xml
        ├── setup.py
        ├── resource/drone_controller
        ├── launch/
        │   ├── controller.launch.py   # NOTE: launches hello_node instances, not `controller`
        │   └── plain_sph.launch.py    # launches one plain_sph node per drone
        └── drone_controller/
            ├── __init__.py
            ├── hello_node.py      # minimal P-controller "go to a fixed target" node
            ├── commander.py       # FullState message builder — see Known issue #3
            ├── robot_model.py     # plain Robot dataclass (position/velocity), no ROS ties
            ├── controller.py      # read-only SPH diagnostics node — BROKEN, see Known issue #6
            ├── sph_logger.py      # CSV logger used by controller.py -> logs/sph_cf_*.csv
            ├── plain_sph.py       # closed-loop SPH controller — this one actually flies the drones
            ├── plain_particle.py  # Particle class with the CMA-ES-tuned constants hardcoded in
            ├── particle.py        # configurable Particle class (gamma/alpha/beta/zetta_extra) — offline use
            ├── raw_sim.py         # pure-Python SPH simulation (no ROS/Gazebo) — fast iteration + plots
            └── CMA_ES.py          # CMA-ES optimizer, tunes gamma/alpha/beta/zetta_extra against raw_sim.py
```

`config/` and `src/` are bind-mounted into the `crazysim` container (see
`docker-compose.yaml`). Editing files on the host takes effect immediately —
`config/` changes need a Gazebo/Crazyswarm2 restart, `src/` changes need a
`colcon build` inside the container (except `raw_sim.py`/`CMA_ES.py`, which
don't need ROS or a build at all — see Track A below).

## First-time setup

```bash
cd ~/SPH_Project
docker compose build crazysim   # ~10-20 min: compiles SITL firmware + colcon-builds crazyswarm2_ws
```

## Every session — always check your prompt

`root@...` = inside the container (correct for all `ros2`/`colcon` commands).
Your own host username = wrong terminal — see Known issue #4.

**After every PC restart, do this first** (the container does not
auto-start on boot — see Known issue #9):
```bash
cd ~/SPH_Project
docker compose up -d crazysim
xhost +local:root

```
Only after this succeeds will `docker compose exec -it crazysim bash` work.
If you skip it, `exec` fails immediately with `service "crazysim" is not
running` — that message means exactly what it says, not a Docker/config
problem.

**Terminal 1 — Gazebo + SITL firmware (headless, default):**
```bash
docker compose exec -it crazysim bash
cd /CrazySim/crazyflie-firmware
bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_multiagent_headless.sh -f agents.txt
```
No GUI window opens — this is deliberate. Physics and firmware behave
identically either way; you just don't get a 3D view. See Known issue #7 if
you want the GUI instead.

**Terminal 2 — Crazyswarm2:**
```bash
docker compose exec -it crazysim bash
source /CrazySim/crazyswarm2_ws/install/setup.bash
ros2 launch crazyflie launch.py backend:=cflib rviz:=True
```

**Terminal 3 — your controller** (note: `src/dev_setup.bash`, NOT plain
`install/setup.bash` — see Known issue #1):
```bash
docker compose exec -it crazysim bash
cd /root/dev_ws
colcon build --symlink-install
source src/dev_setup.bash

# one drone by hand, simple P-controller to a fixed target:
ros2 run drone_controller hello_node --ros-args -p robot_name:=cf_0 -p target:="[0.8, 0.0, 0.5]"

# several drones with the same node:
ros2 launch drone_controller controller.launch.py

# the actual SPH swarm controller (see Track B below):
ros2 launch drone_controller plain_sph.launch.py total_robots:=3
```

## One-off commands

```bash
# Takeoff / land (all drones)
ros2 service call /all/takeoff crazyflie_interfaces/srv/Takeoff "{height: 0.5, duration: {sec: 2, nanosec: 0}}"
ros2 service call /all/land crazyflie_interfaces/srv/Land "{group_mask: 0, height: 0.0, duration: {sec: 2, nanosec: 0}}"

# Hand off from high-level commander to streaming setpoints, per robot
# (see Known issue #2 — durability of this NOT yet confirmed)
ros2 service call /cf_0/notify_setpoints_stop crazyflie_interfaces/srv/NotifySetpointsStop "{group_mask: 0, remain_valid_millisecs: 100}"

# Inspect topics
ros2 topic list | grep state
ros2 topic echo /cf_0/state --once
ros2 topic hz /cf_0/cmd_full_state

# Check colcon's package-type detection
colcon list
```

## Two tracks: offline SPH tuning vs. flying it for real

Both tracks implement the same 2D SPH swarm model (based on Pimenta et al.
2013, "Swarm Coordination Based on Smoothed Particle Hydrodynamics
Technique"), but live in very different environments.

### Track A — Offline tuning (pure Python, no ROS2, no Gazebo)

Files: `particle.py`, `raw_sim.py`, `CMA_ES.py`.

This is where `gamma`, `alpha`, `beta`, and `zetta_extra` (the equation-of-state
exponent and artificial-viscosity coefficients) actually get tuned. It's a
self-contained SPH simulation of an n×n grid of particles — only needs numpy
+ matplotlib, no ROS/Gazebo dependency at all. Because it's fast and
self-contained, this is what `CMA_ES.py` calls internally, hundreds of times,
to score each candidate parameter set during the search.

Run a single simulation with a given parameter set (works outside the
container too, on any machine with numpy + matplotlib):
```bash
cd src/drone_controller/drone_controller
python3 raw_sim.py --gamma 0.0457 --alpha 0.6419 --beta -0.2906 --zetta_extra 0.2905
```
Flags: `--headless` skips the animation window (prints a summary + still shows
the history plots); `--no-history` skips those too. Run `--help` for the full
list (`--n`, `--mass`, `--h`, `--rho_0`, `--dt`, `--num_steps`,
`--expected_distance`).

Run the full CMA-ES search:
```bash
python3 CMA_ES.py
```
Took **~3 hours** on this hardware for 100 generations at the current
settings (8×8 grid, 200 steps per candidate) — there's no interruption/resume
built in yet, so budget the time before starting. Prints per-candidate status
(cost, `ok`/`divergiu`/`parametros invalidos`, timing) and per-generation
summaries as it goes, then shows a 4-panel diagnostic dashboard (cost
convergence, sigma, parameter evolution, covariance condition number) and a
breakdown of the final cost by term.

`h` and `mass` are **not** free parameters here — they're derived
analytically from the drone's sensing radius and target inter-drone spacing
via the SPH kernel-support and equilibrium-density equations (`h =
sensing_radius / 2`; `mass` solved so that `rho(target_spacing) = rho_0`).
Current values, for a 1.5 m sensing radius and 0.5 m target spacing:
`h=0.75`, `mass=795.2156`, `rho_0=1000.0`.

**Current result and a known caveat:** the last full run converged to
`gamma=0.0457, alpha=0.6419, beta=-0.2906, zetta_extra=0.2905` (cost
64.29). Replaying that exact parameter set confirms real convergence — the
closest-neighbor spacing tightened from the 1.0 m initial grid down to
**~0.62 m**, not exactly the 0.5 m target — while the farthest pair stayed
pinned at the original **1.0 m**, almost certainly the grid's outer/corner
particles (fewer neighbors pulling them inward — a standard SPH free-surface
effect, not obviously a tuning failure). No collision risk (spacing never
dropped below target) and the swarm stays well inside the 1.5 m sensing
radius, but it's not a perfectly uniform lattice. Worth another pass with a
higher `spacing_ceiling`/`spacing_floor` weight, or a floor on `gamma`
(currently unconstrained — the optimizer is free to push it toward zero,
which weakens the Tait equation-of-state pressure response), if uniform
spacing matters more than it currently does.

### Track B — In the loop with Gazebo/Crazyswarm2 (real ROS2 nodes)

Files: `plain_particle.py`, `plain_sph.py`, `commander.py` (this is the
working path); separately, `particle.py`, `controller.py`, `sph_logger.py`
(diagnostics path — currently broken, see Known issue #6).

**`plain_sph.py` is the node that actually flies the drones.** One instance
per drone (via `plain_sph.launch.py`), each subscribed to every `/cf_N/state`
topic, each advancing its own SPH particle one step per control tick (0.05s)
using the real measured positions/velocities of all currently-known drones,
then sending the resulting SPH velocity through `Commander.send_velocity`.
It uses `plain_particle.py` — **not** the configurable `particle.py` from
Track A — a frozen copy with the CMA-ES-tuned constants (`h=0.75`,
`gamma=0.0457`, viscosity `[0.6419, -0.2906]`, extra damping `0.2905`)
hardcoded directly into the class, so a ROS launch can never accidentally
fly with stale/default constants.

```bash
# Terminal 1 and 2 as above, then:
ros2 launch drone_controller plain_sph.launch.py total_robots:=3
```

**`controller.py` is a separate, read-only diagnostics node** (subscribes to
state, never publishes any command) meant to log SPH-computed
density/pressure/acceleration as pure telemetry, using the Track-A
`particle.py` and writing through `sph_logger.py` to `logs/sph_cf_*.csv`.
**It currently does not run — see Known issue #6.**

## How to generate history data

Two independent paths, depending on which track you're working in.

**From a real (simulated) flight — CSV + PNG via `plain_sph`:**
1. Start Gazebo/SITL (terminal 1) and Crazyswarm2 (terminal 2) as usual.
2. Launch the SPH controller (terminal 3):
   ```bash
   ros2 launch drone_controller plain_sph.launch.py total_robots:=3
   ```
3. Let it fly for as long as you want data, then stop with **Ctrl+C** —
   `plain_sph.py`'s `destroy_node()` flushes and closes the CSV cleanly on
   shutdown, so a clean interrupt is enough; no need for `kill -9`.
4. CSVs land at `src/logs/plain_sph_cf_<id>.csv` — columns: `t, rho,
   pressure, ext_force_x, ext_force_y, vel_x, vel_y`. Bind-mounted, so
   they're visible on the **host** immediately, no need to stay in the
   container to read them.
5. **On the host**, fix ownership before touching the logs — the container
   writes these as `root`, so your host user can't read or overwrite them
   yet (see Known issue #8):
   ```bash
   sudo chown -R $USER:$USER ~/SPH_Project/src/logs
   ```
6. Generate the plot (works on the host too — only needs `matplotlib`, not
   ROS):
   ```bash
   cd ~/SPH_Project/src
   python3 plot_plain_sph.py
   ```
   Produces `src/logs/plain_sph_metrics.png`: density, pressure, |external
   force|, and |velocity| over time, one line per drone.

Re-running `plain_sph.launch.py` **overwrites** the same CSVs (`'w'` mode,
not append) — copy old ones elsewhere first if you want to keep a specific
run for comparison.

**From an offline simulation — no Gazebo needed, seconds instead of a full flight test:**
```bash
cd src/drone_controller/drone_controller
python3 raw_sim.py --gamma <g> --alpha <a> --beta <b> --zetta_extra <z> --headless
```
Calls `sim.plot_history()` automatically at the end (density, pressure,
min/max velocity over time). Use this first when trying a new parameter set
— it's what `CMA_ES.py` itself uses internally to score every candidate, so
if something looks wrong here, the full ROS/Gazebo flight will show the same
problem, just much slower to discover.

## Known issues / deliberate workarounds

### 1. colcon misidentifies `drone_controller` as `(python)`, not `(ament_python)`
- **Symptom:** `ros2 run` / `ros2 launch` fail with "Package not found" or
  "No executable found" even after a clean `colcon build`.
- **Root cause:** not fully resolved. Installing `python3-catkin-pkg` (already
  added to `docker/sim.Dockerfile`) was a reasonable hypothesis from reading
  `colcon-ros` source, but it did **not** fix the misclassification. Left open
  for a future revisit — not currently blocking.
- **Workaround:** `src/dev_setup.bash` manually does what the `ament_python`
  build task would have done automatically:
  - adds `/root/dev_ws/install/drone_controller` to `AMENT_PREFIX_PATH`
  - symlinks the plain-Python `bin/<exe>` into `lib/drone_controller/<exe>`,
    which is where `ros2 run`/`ros2 launch` actually look.
- **Rule:** always `source src/dev_setup.bash`. Every new `console_scripts`
  entry added to `setup.py` needs one more symlink line added there too
  (currently covers `hello_node`, `controller`, `plain_sph`).

### 2. The high-level commander (takeoff/land/goto) never stands down on its own
- Confirmed by direct test: streaming `cmd_full_state` only "wins" by arriving
  more recently than the high-level commander's own internal setpoint — it
  does not disable it.
- **Proof:** stopping a controller node mid-flight causes an instant snap-back
  toward wherever `takeoff` originally left the drone, not a stay-put.
- `notify_setpoints_stop` exists to hand off control explicitly — **not yet
  confirmed** whether it durably prevents the snap-back or just delays it.
- **Practical rule:** a controller must never stop publishing while it's
  actually flying the drone. Right now, a crash/exception/Ctrl+C means an
  uncontrolled instant setpoint jump, not a safe stop. No watchdog/safe-hold
  behavior exists yet.

### 3. Commander API has moved on again since this note was first written
- `commander.py` no longer has `send_hold` or `send_goal` at all. Current API:
  `send_velocity` (closed-loop from a *desired velocity*: integrates a
  self-consistent position/velocity/acceleration triple, clamped by
  `max_velocity`/`max_acceleration`), `send_accel` (same idea from a *desired
  acceleration*), and the stateless `send_state` underneath both — see the
  docstring at the top of `commander.py`.
- `hello_node.py` uses `send_velocity` with a simple proportional controller
  (`v_des = KP * (target - position)`); `plain_sph.py` uses `send_velocity`
  with the SPH-computed velocity as `v_des`.
- The original problem this section described (a fixed-step command with
  `v=0, a=0` regardless of actual distance, which crashed two drones under
  load) is moot with the current API — there's no fixed-step "hold" method
  left to misuse. Kept here for history.

### 4. Host vs. container terminal mix-ups
- This host has its own ROS2 (Jazzy) and Docker install, separate from
  everything above. Running `ros2`/`colcon` directly on the host produces
  confusing, unrelated errors (wrong `install/`, `Permission denied` under
  `/root`, "Package not found" for a *different* reason than issue #1).
- Always check the prompt before running anything.

### 5. Recovering from a drone crash
- After any hard crash (drone hits the ground hard, or `/cf_x/state` starts
  reporting non-physical values — huge velocities, all-zeros, etc.), fully
  restart Gazebo/SITL and Crazyswarm2 before trying anything else. Don't
  re-takeoff on top of a corrupted physics state.

### 6. `controller.py` currently crashes on startup
- **Symptom:** `TypeError` in `Controller.__init__`, or shortly after in
  `_monitor_step` if construction is patched around.
- **Root cause:** written against a different, richer `Particle` interface
  than the one `particle.py` currently exposes:
  - calls `Particle(..., z=0.0, ...)` — current `Particle.__init__` is 2D
    only, no `z` parameter.
  - calls `p.set_rho(...)` / `p.set_pressure(...)` — current class sets these
    via plain attribute assignment (`p.rho = ...`, `p.P = ...`); no setter
    methods exist.
  - calls `ego_particle.next_vel(neighbor_particles, neighbor_data)` —
    current `next_vel(self, q_list)` takes exactly one argument.
- **Not yet fixed.** Either update `controller.py` to match the current 2D
  `particle.py` (drop `z`, use attribute assignment, drop the second
  `next_vel` argument), or extend `particle.py` with the 3D/setter interface
  `controller.py` expects — depends which direction the diagnostics track is
  meant to go. **`plain_sph.py` is unaffected** — it uses the separate,
  self-consistent `plain_particle.py`.

### 7. `sitl_multiagent_text.sh` (GUI) needs `xhost` on the host first
- The GUI variant of the launch script (`_text.sh`, vs. the default
  `_headless.sh` used in Terminal 1 above) spawns Gazebo's separate GUI
  process (`gz sim -g`), which needs to connect to the host's X server.
- **Symptom if skipped:** physics/firmware start fine (drones spawn,
  handshake succeeds), but the GUI process crashes with `qt.qpa.xcb: could
  not connect to display` / `Authorization required, but no authorization
  protocol specified`, and the script aborts right after.
- **Fix:** on the **host** (not in the container), once per login:
  ```bash
  xhost +local:root
  ```
  `docker-compose.yaml` already mounts `/tmp/.X11-unix` and forwards
  `$DISPLAY` — the missing piece is always the host-side X authorization,
  never the container config.
- **Default recommendation:** just use `_headless.sh` unless you specifically
  need to watch the simulation live.

### 8. `logs/*.csv` and `*.png` come out owned by `root`
- **Symptom:** `python3 plot_plain_sph.py` on the host fails with
  `PermissionError: [Errno 13] Permission denied` when it tries to
  `savefig` into `src/logs/`.
- **Root cause:** `plain_sph.py` (and `controller.py`) run as `root` inside
  the container. `src/logs/` is bind-mounted, so the files it writes are
  owned by `root` on the host too — your normal host user has no write
  access to that directory or its existing files.
- **Fix, on the host, every time before plotting:**
  ```bash
  sudo chown -R $USER:$USER ~/SPH_Project/src/logs
  ```
- This isn't a one-time fix — every fresh run of `plain_sph.launch.py`
  re-creates the CSVs as `root` again, so re-run the `chown` each session
  before `plot_plain_sph.py`. Already folded into "How to generate history
  data" above.

### 9. Container doesn't come back on its own after a PC restart
- **Symptom:** `docker compose exec -it crazysim bash` immediately says
  `service "crazysim" is not running` — no container, no error from Docker
  itself, just a plain refusal.
- **Root cause:** `docker-compose.yaml` has no `restart:` policy set, so the
  container is not brought back up automatically on boot/Docker daemon
  restart. This has nothing to do with the image or the build — it's simply
  not running yet.
- **Fix, every session after a reboot, before anything else:**
  ```bash
  cd ~/SPH_Project
  docker compose up -d crazysim
  ```
  Then `docker compose exec -it crazysim bash` works as normal. Already
  folded into "Every session" above.
- **If `up -d` itself fails or the container exits immediately**, that's a
  different problem — check `docker compose ps -a` (look for `Exited`) and
  `docker compose logs crazysim` for the actual error before assuming it's
  this issue.

## Multi-drone config

Two files must stay in sync, same order, same count:
- `config/crazyflies.yaml` — robot list, `uri: udp://127.0.0.1:1985<N>`,
  `initial_position`.
- `config/agents.txt` — matching `x, y` spawn list.

`custom_topics` (not `default_topics`) is what makes `/cf_N/state` exist —
easy to get wrong silently (no error, the topic just never appears). Use the
existing `crazyflies.yaml` as the template rather than reconstructing it from
memory.

## Project status

| Step | What | Status |
|---|---|---|
| 1 | Environment: Docker, Gazebo, SITL, Crazyswarm2 | ✅ Done, verified |
| 2 | Own ROS2 package builds and runs | ✅ Done (with colcon workaround) |
| 3 | Node reads real drone state | ✅ Done, verified against known-good baseline |
| 4 | Node sends a command | ✅ Done; commander API refactored since — see Known issue #3 |
| 5 | Multiple drone instances via one launch file | ✅ Done, verified 3 drones holding simultaneously |
| 6 | Real move-and-hold trajectory | ✅ `hello_node` closed-loop via `send_velocity` (simple P-controller) |
| 7 | Offline SPH parameter tuning (CMA-ES) | ✅ Done — see Track A; converged, with the spacing caveat noted there |
| 8 | SPH controller actually flying drones | ✅ `plain_sph.py` working end-to-end, logging, plottable |
| 9 | Read-only SPH diagnostics node (`controller.py`) | 🔴 Broken — see Known issue #6 |

**Open/parked items:** colcon `ament_python` misclassification root cause
(#1); `notify_setpoints_stop` durability (#2); `controller.py`/`particle.py`
interface mismatch (#6); non-uniform equilibrium spacing in the current
CMA-ES result (see Track A). Two routine (not blocking, just annoying)
per-session steps until automated away: log ownership after every
`plain_sph` run (#8), and bringing the container back up after every PC
restart (#9) — both could be scripted into one `./dev_up.sh`-style helper
later if they get tiresome enough.