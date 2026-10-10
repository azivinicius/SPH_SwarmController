# SPH_Project

SPH (smoothed-particle) swarm controller for Crazyflies. A ROS2 package (`drone_controller`) that runs
inside one Docker container with CrazySim (Gazebo + firmware) and Crazyswarm2. Each drone is an SPH
particle; an external potential pulls the swarm onto a target shape.

Everything is started from the host with `./dev_tmux.sh` (one tmux session, every command runs inside
the `crazysim` container). Run it from `~/SPH_Project`.

## 1. First time

```bash
cd ~/SPH_Project
sudo apt install tmux
chmod +x dev_tmux.sh gen_config.py
docker compose build crazysim        # 10-20 min
```

## 2. Boost the PC (do this after every reboot, laptop on the charger)

The CPU profile goes back to `power` at every boot and then runs at ~1 GHz (RTF 0.5 with 3 drones).
With `performance` the clock goes to 2.3-2.7 GHz (RTF ~0.94).

```bash
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/energy_performance_preference
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
```

Check (must print `performance` twice) and watch the clock:

```bash
cat /sys/devices/system/cpu/cpu0/cpufreq/energy_performance_preference /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
grep MHz /proc/cpuinfo
```

It only matters for CrazySim (`up`, `up-swarm`), where RTF must stay >= 0.9. `up-particles` has no
physics and runs fine without it. After a reboot also start the container: `docker compose up -d crazysim`
(`./dev_tmux.sh up` does it for you).

## 3. Commands

```bash
./dev_tmux.sh up              # CrazySim (Gazebo + firmware) + Crazyswarm2 + one controller node per drone
./dev_tmux.sh up-swarm        # same, but ONE controller process for all drones
./dev_tmux.sh up-particles    # NO physics: particles move in Gazebo, controller runs on top (fast)
./dev_tmux.sh show            # print the resolved config and launch args, start nothing
./dev_tmux.sh check           # host config files vs what the container sees
./dev_tmux.sh analyze         # numbers of the last run -> src/logs/runs/<id>/analysis/summary.txt
./dev_tmux.sh stop            # stop everything (same as Ctrl+b then K inside tmux)
./dev_tmux.sh attach          # re-attach to a running session
```

Every knob is an environment variable (defaults in `swarm.conf`). Examples:

#>>>>
N_DRONES=3 SPH_MASS=auto SPH_H=0.3 SPH_SPACING=0.3 SPH_GAIN=0.05 ./dev_tmux.sh up

N_DRONES=5 SPH_MASS=auto SPH_H=0.3 SPH_SPACING=0.5 SPH_GAIN=0.05 POT_K=2.0 ./dev_tmux.sh up

N_DRONES=5 POTENTIAL=quartic SPH_MASS=auto SPH_H=0.3 SPH_SPACING=0.5 SPH_GAIN=0.05 POT_K=1.0 ./dev_tmux.sh up

./dev_tmux.sh stop
N_DRONES=5 SPH_MASS=auto SPH_DIM=3 POT_K=4.0 SPH_GAIN=0.05 SPH_SPACING=0.5 ./dev_tmux.sh up

./dev_tmux.sh stop
N_DRONES=5 SPH_MASS=auto SPH_DIM=3 POTENTIAL=quartic POT_K=1.0 SPH_GAIN=0.05 SPH_SPACING=0.5 ./dev_tmux.sh up

N_DRONES=20 SPH_MASS=auto POT_R=auto POTENTIAL=quartic SPH_H=0.3 SPH_SPACING=0.3 SPH_GAIN=0.05 ./dev_tmux.sh up-particles

N_DRONES=20 SPH_MASS=auto POT_R=auto POTENTIAL=quartic SPH_H=0.3 SPH_SPACING=0.3 SPH_GAIN=0.05 ./dev_tmux.sh up-particles

N_DRONES=8 SPH_DIM=3 SPH_MASS=auto POT_R=auto SPH_GAIN=0.05 ./dev_tmux.sh up-particles


#>>>>

If a session is already open: `./dev_tmux.sh stop` first. Label a run with `RUN_LABEL=name`.

### Main knobs

| Variable | Meaning |
|---|---|
| `N_DRONES` | number of drones |
| `SPH_MASS` | particle mass, or `auto` = solved so neighbours sit `SPH_SPACING` apart (2D circle/ring and 2D quartic; 3D `auto` is for the sphere only) |
| `SPH_H`, `SPH_SPACING` | smoothing length and wanted neighbour spacing (m) |
| `SPH_GAIN` | multiplies only the SPH force (0 = SPH off) |
| `POTENTIAL` | `default` (circle in 2D, sphere in 3D) or `quartic` |
| `POT_XC`, `POT_YC`, `POT_ZC` | centre of the target |
| `POT_R` | circle/sphere radius (`auto` = the spawn radius). For the quartic it is a size constant, not a radius: perimeter ~ 6.95 * R^(1/4) |
| `POT_K` | circle/sphere: force strength. Quartic: strength factor = `POT_K / 2` (2 = original), the cap `max_force` stays |
| `SPH_DIM` | 2 (default) or 3 |
| `FIXED_Z`, `MAX_VELOCITY` | flight height (2D) and speed limit of the command (m/s) |
| `SPAWN_LAYOUT` | `grid` or `ring` for CrazySim runs (`up-particles` starts in a grid) |

3D extras: `MAX_VZ`, `Z_MIN`, `Z_MAX`, `Z_MARGIN`, `Z_WALL_K`, `Z_STRETCH`.
`up-particles` extras: `PART_CTRL` (`plain` = one node per drone, `swarm`), `PART_LAYOUT`, `PART_TAU`,
`PART_DELAY`, `PART_VIZ` (`gz|rviz|none`), `GZ_GUI`.

### Quartic potential: what to know

- Only the quartic changes with `POTENTIAL=quartic`; circle/sphere are unchanged.
- It is stiff. Large `POT_R` (about 4-5 and up) chatters in the 20 Hz loop; small `POT_R` makes the force
  weak, so raise `POT_K`. Starting point for 5 drones at 0.3 m: `POT_R=0.012 POT_K=100`.
- 3D: the loop lies on the plane `z = (x + y) / 2` (it does not follow `POT_ZC`).
- `POT_R=auto` gives a circle radius: do not use it with the quartic.

## 4. Plots

```bash
python3 src/plot_plain_sph.py                      # newest run
python3 src/plot_plain_sph.py src/logs/runs/<id>   # a given run
python3 src/plot_plain_sph.py --no-open            # only write the PNGs
```

The PNGs go to `src/logs/runs/<id>/plots/`. Each one opens in its own window and the script keeps running;
press Ctrl+C in the terminal to close them all. It uses `feh` if present (`sudo apt install feh`), else
`eog` / `display`; `PLOT_VIEWER="cmd args"` picks another viewer.

| File | Content |
|---|---|
| `00_summary` | parameters and key results |
| `01_trajectories` | paths + target, altitude, formation snapshots |
| `02_separation` | closest approach, collisions, spacing histogram |
| `03_sph_dynamics` | rho/rho0, pressure acceleration, external acceleration, speed |
| `04_performance` | RTF, CPU load per process group, clock and temperature |
| `05_3d` | 3D runs only |

## 5. Stopping

`Ctrl+b` then `K` (inside tmux) or `./dev_tmux.sh stop` from any terminal. It stops the controller first
(so the CSVs are flushed), kills the leftovers in the container and fixes the ownership of the logs.
Detach without stopping: `Ctrl+b` then `d`. Windows: `Ctrl+b` then `0..3` (`sim`, `swarm`, `ctrl`, `shell`).

## 6. Where things are

```
dev_tmux.sh, swarm.conf      launcher and every run knob
gen_config.py                writes config/agents.txt, config/crazyflies.yaml (and the particle world)
config/                      generated drone list + Crazyswarm2 config (bind-mounted, edited in place)
docker/, docker-compose.yaml container
src/plot_plain_sph.py        plots;  src/analyze_run.py  numbers;  src/tools/  RTF and CPU loggers
src/logs/runs/<id>/          one folder per run (tick/rx CSVs, events.log, rtf.csv, sys.csv, meta.json, plots/)
src/drone_controller/        the ROS2 package
  sph_core.py                the control loop (ROS-free)        potential.py / potential3d.py  targets
  particle.py / particle3d.py  SPH particle (2D / 3D)          particle_sim.py  no-physics simulator
  plain_sph.py / swarm_sph.py  ROS nodes (one per drone / one for all)
REAL_DRONES.md               checklist for real Crazyflies
```

`src/` is bind-mounted: after editing Python files in `src/`, `dev_tmux.sh up*` rebuilds (`colcon build`)
for you. `config/` changes need a restart.

## 7. Problems that already happened

- **`ros2 run ... No executable found`**: every executable needs an entry in
  `src/drone_controller/setup.py` (`console_scripts`) AND a symlink block in `src/dev_setup.bash`
  (colcon does not classify the package as `ament_python` here, so `lib/drone_controller/` is filled by hand).
- **Run folder owned by root** (plot cannot write): `./dev_tmux.sh stop` fixes it, or
  `sudo chown -R $USER:$USER ~/SPH_Project/src/logs`.
- **Container not running after a reboot**: `docker compose up -d crazysim`.
- **`ros2: command not found` in `docker compose exec ... bash -lc`**: source
  `/opt/ros/humble/setup.bash` and `/CrazySim/crazyswarm2_ws/install/setup.bash` first (`dev_tmux.sh` does).
- **`pkill -f name` inside `bash -lc` kills its own shell**: use `pkill -f '[n]ame'`.
- **Leftover processes after a crash**: `docker compose exec crazysim ps aux | grep -E 'gz|cf2|ruby|ros2'`.
- **Low RTF**: do section 2; close other heavy programs; read `04_performance.png`. Below 0.6 do not
  trust the result. `SIM_CLOCK=gz` makes the controller follow simulated time and ignore the RTF.
- **Recovering a crashed drone in CrazySim**: `./dev_tmux.sh stop`, then `up` again.