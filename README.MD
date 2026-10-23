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
GZ_GUI=1 ./dev_tmux.sh up

#>>>>
N_DRONES=5 SPH_MASS=auto SPH_H=0.3 SPH_SPACING=0.3 SPH_GAIN=0.05 ./dev_tmux.sh up

N_DRONES=5 POTENTIAL=quartic SPH_MASS=auto SPH_H=0.3 SPH_SPACING=0.5 SPH_GAIN=0.5 POT_K=4.0 ./dev_tmux.sh up

./dev_tmux.sh stop
N_DRONES=5 SPH_MASS=auto SPH_DIM=3 POT_K=4.0 SPH_GAIN=0.05 SPH_SPACING=0.5 ./dev_tmux.sh up

./dev_tmux.sh stop
N_DRONES=5 SPH_MASS=auto SPH_DIM=3 POTENTIAL=quartic POT_K=1.0 SPH_GAIN=0.05 SPH_SPACING=0.5 ./dev_tmux.sh up

N_DRONES=20 SPH_MASS=auto POT_R=auto POTENTIAL=quartic SPH_H=0.3 SPH_SPACING=0.3 SPH_GAIN=0.05 ./dev_tmux.sh up-particles

N_DRONES=20 SPH_MASS=auto POT_R=auto POTENTIAL=quartic SPH_H=0.3 SPH_SPACING=0.3 SPH_GAIN=0.05 ./dev_tmux.sh up-particles

N_DRONES=8 SPH_DIM=3 SPH_MASS=auto POT_R=auto SPH_GAIN=0.05 ./dev_tmux.sh up-particles

# 3 drones, one obstacle beside the straight path
N_DRONES=3 SPH_MASS=auto SPH_H=0.3 SPH_SPACING=0.3 SPH_GAIN=0.05 OBSTACLES='1.0,1.3' RUN_LABEL=obs1 ./dev_tmux.sh up

# 5 drones through a "gate": two obstacles with a gap on the straight path
N_DRONES=5 SPH_MASS=auto SPH_H=0.3 SPH_SPACING=0.5 SPH_GAIN=0.05 POT_K=2.0 OBSTACLES='0.5,1.7/1.5,0.5' RUN_LABEL=gate ./dev_tmux.sh up

# ---------- base comum (rode uma vez no terminal) ----------
export N_DRONES=5 SPH_MASS=auto SPH_H=0.3 SPH_SPACING=0.3 SPH_GAIN=0.5

# ---------- PARTÍCULAS (sem física) ----------
# 1. potencial circular
RUN_LABEL=p_circ ./dev_tmux.sh up-particles
# 1.1 potencial circular (muitas partículas)    
N_DRONES=20 POT_K=10.0 POT_R=0.96 SPH_GAMMA=3.0 SPH_SPACING=0.3 SPH_ZETTA=50.0 RUN_LABEL=p_circ_20 ./dev_tmux.sh up-particles
# 2. quártico 2D
N_DRONES=12 POTENTIAL=quartic POT_K=10.0 POT_R=0.23 SPH_GAMMA=3.0 SPH_SPACING=0.4 SPH_ZETTA=50.0 RUN_LABEL=p_quart2d ./dev_tmux.sh up-particles

N_DRONES=5 POTENTIAL=quartic POT_K=20.0 POT_R=0.035 SPH_GAMMA=3.0 SPH_SPACING=0.6 SPH_ZETTA=50.0 RUN_LABEL=p_quart2d ./dev_tmux.sh up-particles
N_DRONES=6 POTENTIAL=quartic POT_K=20.0 POT_R=0.072 SPH_GAMMA=3.0 SPH_SPACING=0.6 SPH_ZETTA=50.0 RUN_LABEL=p_quart2d ./dev_tmux.sh up-particles

# 3. esférico
N_DRONES=8 RUN_LABEL=p_esfera POT_K=2.0 POT_R=0.41 SPH_SPACING=0.5 SPH_DIM=3 POT_ZC=1.2 ./dev_tmux.sh up-particles
N_DRONES=12 RUN_LABEL=p_esfera POT_K=2.0 POT_R=0.29 SPH_SPACING=0.3 SPH_DIM=3 POT_ZC=0.7 ./dev_tmux.sh up-particles

# 4. quártico 3D
N_DRONES=8 SPH_GAMMA=3.0 RUN_LABEL=p_quart3d SPH_DIM=3 POTENTIAL=quartic POT_R=0.23 POT_XC=1.5 POT_YC=1.5 POT_K=10.0 SPH_MASS=330 SPH_H=0.5 SPH_SPACING=0.5 ./dev_tmux.sh up-particles

N_DRONES=12 SPH_GAMMA=3.0 RUN_LABEL=p_quart3d SPH_DIM=3 POTENTIAL=quartic POT_R=0.1 POT_XC=1.5 POT_YC=1.5 POT_K=20.0 SPH_MASS=70.0 SPH_H=0.3 SPH_SPACING=0.3 SPH_GAIN=0.5 ./dev_tmux.sh up-particles

# 5. quártico tangente 2D
N_DRONES=12 POTENTIAL=quartic POT_K=10.0 POT_R=0.23 SPH_GAMMA=3.0 SPH_SPACING=0.4 SPH_ZETTA=50.0 POT_KT=1.0 RUN_LABEL=p_quart2d_tang ./dev_tmux.sh up-particles
# 6. quártico tangente 3D     
N_DRONES=8 SPH_GAMMA=3.0 RUN_LABEL=p_quart3d_tang SPH_DIM=3 POTENTIAL=quartic POT_R=0.23 POT_XC=1.5 POT_YC=1.5 POT_K=10.0 SPH_MASS=330 SPH_H=0.5 SPH_SPACING=0.5 POT_KT=1.0 ./dev_tmux.sh up-particles

# ---------- DRONES (CrazySim) ----------
# ---------- base comum (rode uma vez no terminal) ----------
export N_DRONES=5 SPH_MASS=auto SPH_H=0.3 SPH_SPACING=0.3 SPH_GAIN=0.5

# 1. potencial circular
RUN_LABEL=cs_circ ./dev_tmux.sh up
# 1.1 potencial circular, 8 drones                               [20 -> 8: R = 0,3 / (2 sen(pi/8)) = 0,39]
N_DRONES=8 POT_K=10.0 POT_R=0.39 SPH_GAMMA=3.0 SPH_SPACING=0.3 SPH_ZETTA=50.0 RUN_LABEL=cs_circ_8 ./dev_tmux.sh up

# 2. quártico 2D                                                  [12 -> 6: ver nota 1]
N_DRONES=6 POTENTIAL=quartic POT_K=20.0 POT_R=0.072 SPH_GAMMA=3.0 SPH_SPACING=0.6 SPH_H=0.5 SPH_ZETTA=50.0 RUN_LABEL=cs_quart2d_6 ./dev_tmux.sh up
N_DRONES=5 POTENTIAL=quartic POT_K=20.0 POT_R=0.035 SPH_GAMMA=3.0 SPH_SPACING=0.6 SPH_H=0.5 SPH_ZETTA=50.0 RUN_LABEL=cs_quart2d_5 ./dev_tmux.sh up

# 3. esférico
N_DRONES=8 RUN_LABEL=cs_esfera_8 POT_K=2.0 POT_R=0.41 SPH_SPACING=0.5 SPH_DIM=3 POT_ZC=1.2 ./dev_tmux.sh up
#                                                                 [12 -> 6: octaedro, R = 0,3 / 1,41 = 0,21]
N_DRONES=6 RUN_LABEL=cs_esfera_6 POT_K=2.0 POT_R=0.21 SPH_SPACING=0.3 SPH_DIM=3 POT_ZC=0.7 ./dev_tmux.sh up

# 4. quártico 3D
N_DRONES=8 SPH_GAMMA=3.0 RUN_LABEL=cs_quart3d_8 SPH_DIM=3 POTENTIAL=quartic POT_R=0.23 POT_XC=1.5 POT_YC=1.5 POT_K=10.0 SPH_MASS=330 SPH_H=0.5 SPH_SPACING=0.5 ./dev_tmux.sh up
#                                                                 [12 -> 6: ver nota 2]
N_DRONES=6 SPH_GAMMA=3.0 RUN_LABEL=cs_quart3d_6 SPH_DIM=3 POTENTIAL=quartic POT_R=0.07 POT_XC=1.5 POT_YC=1.5 POT_K=60.0 SPH_MASS=330 SPH_H=0.5 SPH_SPACING=0.5 ./dev_tmux.sh up

# 5. quártico tangente 2D                                         [12 -> 6: ver notas 1 e 3]
N_DRONES=6 POTENTIAL=quartic POT_K=20.0 POT_R=0.072 SPH_GAMMA=3.0 SPH_SPACING=0.6 SPH_H=0.5 SPH_ZETTA=50.0 POT_KT=2.0 RUN_LABEL=cs_quart2d_tang_6 ./dev_tmux.sh up
# 6. quártico tangente 3D (já era 8)
N_DRONES=8 SPH_GAMMA=3.0 RUN_LABEL=cs_quart3d_tang_8 SPH_DIM=3 POTENTIAL=quartic POT_R=0.23 POT_XC=1.5 POT_YC=1.5 POT_K=10.0 SPH_MASS=330 SPH_H=0.5 SPH_SPACING=0.5 POT_KT=1.0 ./dev_tmux.sh up

# ---------- OBSTÁCULOS (CrazySim) ----------
export N_DRONES=5 SPH_MASS=auto SPH_H=0.3 SPH_SPACING=0.3 SPH_GAIN=0.5

# ---------- 13. obstáculo 2D sobre o círculo (R = 0,8), no lado de chegada (225°) ----------
N_DRONES=8 POT_R=0.8 POT_K=10.0 SPH_GAMMA=3.0 SPH_ZETTA=50.0 OBSTACLES="1.43,1.43" RUN_LABEL=p_obs2d ./dev_tmux.sh up-particles
N_DRONES=8 POT_R=0.8 POT_K=10.0 SPH_GAMMA=3.0 SPH_ZETTA=50.0 OBSTACLES="1.43,1.43" RUN_LABEL=cs_obs2d ./dev_tmux.sh up

# ---------- 14. obstáculo 3D sobre a esfera (R = 0,8), no ponto voltado para a decolagem ----------
N_DRONES=8 SPH_DIM=3 POT_ZC=1.4 POT_R=0.8 POT_K=2.0 OBSTACLES="1.46,1.46,1.16" RUN_LABEL=p_obs3d ./dev_tmux.sh up-particles
N_DRONES=8 SPH_DIM=3 POT_ZC=1.4 POT_R=0.8 POT_K=2.0 OBSTACLES="1.46,1.46,1.16" RUN_LABEL=cs_obs3d ./dev_tmux.sh up

# ---------- 15. obstáculo próximo à estrela, com circulação ----------
N_DRONES=5 POTENTIAL=quartic POT_R=1.0 POT_KT=4.5 OBSTACLES="2.0,2.85" OBS_SIGMA=2.0 RUN_LABEL=p_obs_estrela_tang ./dev_tmux.sh up-particles
N_DRONES=5 POTENTIAL=quartic POT_R=1.0 POT_KT=4.5 OBSTACLES="2.0,2.85" OBS_SIGMA=2.0 RUN_LABEL=cs_obs_estrela_tang ./dev_tmux.sh up

# ---------- 16. estrela sem circulação, obstáculo no meio do lado de baixo (lado de chegada) ----------
N_DRONES=12 POTENTIAL=quartic POT_K=10.0 POT_R=0.23 SPH_GAMMA=3.0 SPH_SPACING=0.4 SPH_ZETTA=50.0 OBSTACLES="2.0,1.51" RUN_LABEL=p_obs_estrela ./dev_tmux.sh up-particles

./dev_tmux.sh stop
# pillar (x,y = full-height column) + one obstacle in the air (x,y,z)
N_DRONES=5 SPH_MASS=auto SPH_DIM=3 POT_K=4.0 SPH_GAIN=0.05 SPH_SPACING=0.5 OBSTACLES='0.5,1.5/1.5,0.5,0.8' RUN_LABEL=obs3d ./dev_tmux.sh up
  
./dev_tmux.sh stop
N_DRONES=5 SPH_MASS=auto SPH_DIM=3 POTENTIAL=quartic POT_K=1.0 SPH_GAIN=0.05 SPH_SPACING=0.5 OBSTACLES='0.5,1.5' RUN_LABEL=star3d_pillar ./dev_tmux.sh up

# 20 particles, star shape: two short walls on either side of the path
N_DRONES=20 SPH_MASS=auto POT_R=auto POTENTIAL=quartic SPH_H=0.3 SPH_SPACING=0.3 SPH_GAIN=0.05 OBSTACLES='0.0,1.5/0.3,1.5/0.6,1.5/1.5,0.0/1.5,0.3/1.5,0.6' RUN_LABEL=walls ./dev_tmux.sh up-particles

# 20 particles, star shape, obstacle in the centre of the star
N_DRONES=20 SPH_MASS=auto POT_R=auto POTENTIAL=quartic SPH_H=0.3 SPH_SPACING=0.3 SPH_GAIN=0.05 OBSTACLES='2.0,2.0' RUN_LABEL=star_centre ./dev_tmux.sh up-particles

# 8 particles, 3D: pillar + obstacle in the air
N_DRONES=8 SPH_DIM=3 SPH_MASS=auto POT_R=auto SPH_GAIN=0.05 OBSTACLES='0.5,1.5/1.5,0.5,0.8' RUN_LABEL=obs3d_p ./dev_tmux.sh up-particles

RVIZ=False GZ_GUI=1 N_DRONES=5 SPH_MASS=auto SPH_H=0.3 SPH_SPACING=0.5 SPH_GAIN=0.05 POT_K=2.0 ./dev_tmux.sh up



export N_DRONES=5 SPH_H=0.3 SPH_SPACING=0.5 SPH_GAIN=0.5 OBS_SIGMA=2.0

export SPH_H=0.3 SPH_SPACING=0.5 SPH_GAIN=0.5 OBS_SIGMA=2.0

# ---------- 1. círculo com portão (centros a 1,6 m: canal livre de 0,4 m) ----------
N_DRONES=6 SPH_MASS=auto SPH_GAMMA=3.0 SPH_ZETTA=50.0 POT_K=10.0 POT_R=0.50 OBSTACLES="0.65,1.35/1.35,0.65" RUN_LABEL=p_obs_circ_portao_6 ./dev_tmux.sh up-particles
N_DRONES=6 SPH_MASS=auto SPH_GAMMA=3.0 SPH_ZETTA=50.0 POT_K=10.0 POT_R=0.50 OBSTACLES="0.65,1.35/1.35,0.65" RUN_LABEL=cs_obs_circ_portao_6 ./dev_tmux.sh up
N_DRONES=3 SPH_MASS=auto SPH_GAMMA=3.0 SPH_ZETTA=50.0 POT_K=10.0 POT_R=0.29 OBSTACLES="0.65,1.35/1.35,0.65" RUN_LABEL=cs_obs_circ_portao_3 ./dev_tmux.sh up
# ---------- 2. esfera, obstáculo parcialmente sobre ela, fora da reta de chegada ----------
# 6 drones: octaedro, R = 0,5 / 1,41 = 0,36 ; obstáculo no equador, lado sudeste, a 0,51 m do centro
N_DRONES=6 SPH_DIM=3 SPH_MASS=auto SPH_GAMMA=7.0 SPH_ZETTA=40.0 POT_K=2.0 POT_R=0.36 POT_ZC=1.2 OBSTACLES="2.36,1.64,1.2" RUN_LABEL=p_obs_esfera_6 ./dev_tmux.sh up-particles
N_DRONES=6 SPH_DIM=3 SPH_MASS=auto SPH_GAMMA=7.0 SPH_ZETTA=40.0 POT_K=2.0 POT_R=0.36 POT_ZC=1.2 OBSTACLES="2.36,1.64,1.2" RUN_LABEL=cs_obs_esfera_6 ./dev_tmux.sh up
N_DRONES=3 SPH_DIM=3 SPH_MASS=auto SPH_GAMMA=7.0 SPH_ZETTA=40.0 POT_K=2.0 POT_R=0.29 POT_ZC=1.2 OBSTACLES="2.31,1.69,1.2" RUN_LABEL=cs_obs_esfera_3 ./dev_tmux.sh up

# ---------- 3. curva quártica 3D, obstáculo sobre o lado sul ----------
# 6 drones: R = 0,0725 (curva de ~4,0 m); lados a 0,37 m, vértices a 0,62 m; altitude 1,06–1,94 m
N_DRONES=6 SPH_DIM=3 POTENTIAL=quartic SPH_MASS=70 SPH_GAMMA=3.0 SPH_ZETTA=40.0 POT_K=10.0 POT_R=0.11 POT_XC=1.5 POT_YC=1.5 OBSTACLES="1.5,0.94,1.30" RUN_LABEL=p_obs_quart3d_6 ./dev_tmux.sh up-particles
N_DRONES=6 SPH_DIM=3 POTENTIAL=quartic SPH_MASS=70 SPH_GAMMA=3.0 SPH_ZETTA=40.0 POT_K=10.0 POT_R=0.11 POT_XC=1.5 POT_YC=1.5 OBSTACLES="1.5,0.94,1.30" RUN_LABEL=cs_obs_quart3d_6 ./dev_tmux.sh up
N_DRONES=3 SPH_DIM=3 POTENTIAL=quartic SPH_MASS=150 SPH_GAMMA=3.0 SPH_ZETTA=40.0 POT_K=40.0 POT_R=0.011 POT_XC=1.5 POT_YC=1.5 OBSTACLES="1.5,1.12,1.39" RUN_LABEL=cs_obs_quart3d_3 ./dev_tmux.sh up

# ---------- 4. curva quártica 2D com circulação, obstáculo sobre o lado norte ----------
# 6 drones: R = 0,110 (curva de ~4,0 m); lados a 0,41 m, vértices a 0,69 m
N_DRONES=6 POTENTIAL=quartic SPH_MASS=auto SPH_GAMMA=3.0 SPH_ZETTA=50.0 POT_K=15.0 POT_KT=7.0 POT_R=0.4 OBSTACLES="2.0,2.76" RUN_LABEL=p_obs_quart2d_tang_6 ./dev_tmux.sh up-particles
N_DRONES=6 POTENTIAL=quartic SPH_MASS=auto SPH_GAMMA=3.0 SPH_ZETTA=50.0 POT_K=15.0 POT_KT=7.0 POT_R=0.4 OBSTACLES="2.0,2.76" RUN_LABEL=cs_obs_quart2d_tang_6 ./dev_tmux.sh up
N_DRONES=3 POTENTIAL=quartic SPH_MASS=auto SPH_GAMMA=3.0 SPH_ZETTA=50.0 POT_K=15.0 POT_KT=7.0 POT_R=0.4 OBSTACLES="2.0,2.76" RUN_LABEL=cs_obs_quart2d_tang_6 ./dev_tmux.sh up

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
## Testes

Há dois tipos de teste:

- **Testes de movimento** (`src/drone_controller/testes/`): scripts Python simples, sem o controlador SPH.
  Decolam pelo serviço `/cf_i/takeoff`, movimentam com `Commander.send_velocity_world()` e pousam com
  `Commander.land()`.
- **Testes SPH**: o controlador `plain_sph` com conjuntos de parâmetros dimensionados para a arena real.

Arena considerada: cerca de ±1,5 m em x e y (centro em (0, 0)), |v| ≤ 0,2 m/s, |vz| ≤ 0,2 m/s, decolagem a 0,5 m.

### 1. Testes de movimento

| Script | Movimento |
|---|---|
| `frente_tras.py` | +x por `--tempo` s, pausa, −x por `--tempo` s, pausa |
| `frente_tras_lados.py` | frente (+x), trás (−x), lado (+y), outro lado (−y por 2×`--tempo`), volta ao centro (+y) |

Argumentos comuns: `--v` (m/s, padrão 0.2), `--tempo` (s, padrão 3), `--pausa` (s, padrão 2), `--repeticoes`,
`--height` (m, padrão 0.5), `--fence XMIN XMAX YMIN YMAX` (padrão −1.5 1.5 −1.5 1.5), `--kz` (segura a altura, padrão 0).
Ajuda: `./dev_tmux.sh test frente_tras --help`.

Segurança: se um drone sai da cerca ou fica 1 s sem `/state`, todos recebem 0 m/s e pousam.
**Ctrl+C pousa os drones** (um segundo Ctrl+C sai sem pousar).

Logs: `src/logs/testes/<data>_<teste>.csv` (posição, velocidade e comando por drone, por fase) e `.json` com os argumentos.

#### No CrazySim

```bash
./dev_tmux.sh test                                     # lista os testes
./dev_tmux.sh test frente_tras --v 0.2 --tempo 3
./dev_tmux.sh test frente_tras_lados --v 0.2 --tempo 3
./dev_tmux.sh stop                                     # ao terminar
```

A primeira chamada sobe o CrazySim **sem** o controlador SPH e espera o `crazyflie_server` (1–2 min).
As seguintes reaproveitam a simulação. Número de drones: `N_DRONES=3 ./dev_tmux.sh test ...` (só vale ao subir a simulação).

#### Nos drones reais

```bash
./real_tmux.sh up                                      # Crazyswarm2 com config/crazyflies_real.yaml
./real_tmux.sh test frente_tras --v 0.2 --tempo 2      # em outro terminal (ou Ctrl+b d)
./real_tmux.sh test frente_tras_lados --v 0.2 --tempo 2
./real_tmux.sh stop
```

- Os drones habilitados no yaml devem ser `cf_0 .. cf_{N-1}`.
- Comece com 1 drone e `--tempo 2` (0,4 m de deslocamento).
- O comando `test` se recusa a rodar se o controlador SPH (janela `ctrl`) estiver aberto.

#### Direto no container

```bash
docker compose exec crazysim bash
source /opt/ros/humble/setup.bash && source /CrazySim/crazyswarm2_ws/install/setup.bash
source /root/dev_ws/src/dev_setup.bash
cd /root/dev_ws/src/drone_controller/testes
python3 frente_tras.py --v 0.2 --tempo 3
```

### 2. Testes SPH

Os testes SPH rodam com `./real_tmux.sh sph` (real) ou `./dev_tmux.sh up` (CrazySim).
**Não use `./real_tmux.sh ctrl` para o SPH**: ele força `sph_gain:=0` e `f_k:=0` (é só para perfis de teste).

O `swarm.conf` precisa usar `VAR=${VAR:-valor}` para que as variáveis abaixo tenham efeito.
Conferir: `POT_K=1.234 ./dev_tmux.sh show | grep -o "f_k:=[^ ]*"` deve mostrar `f_k:=1.234`.

#### Parâmetros comuns (defina uma vez no terminal)

```bash


As variáveis de cada teste vêm depois de `$SPH_REAL` e substituem as comuns (por exemplo `Z_MIN` no 3D).

### Posições de decolagem

Coordenadas x, y em metros, com tolerância de ±5 cm, no referencial do sistema de posicionamento (centro da arena em (0, 0)).
Os pontos ficam sobre a curva de cada teste, distribuídos igualmente ao longo dela. Perto do centro a força
quártica é quase nula, e um drone que decola ali pode ficar preso. Nos testes 3D, o laço fica centrado em (0,7, 0,7),
porque o plano do potencial, `z = (x + y)/2`, passa pela origem.

| Teste | cf_0 | cf_1 | cf_2 | cf_3 | cf_4 |
|---|---|---|---|---|---|
| circ 3 | (+0.32, +0.13) | (-0.28, +0.21) | (-0.04, -0.34) | | |
| quart 2D 4 | (+0.26, +0.11) | (-0.11, +0.26) | (-0.26, -0.11) | (+0.11, -0.26) | |
| quart 2D 5 / quart 2D tang 5 | (+0.33, +0.14) | (+0.01, +0.31) | (-0.33, +0.16) | (-0.30, -0.37) | (+0.28, -0.37) |
| quart 3D 4 | (+0.96, +0.81) | (+0.59, +0.96) | (+0.44, +0.59) | (+0.81, +0.44) | |
| quart 3D 5 | (+1.03, +0.84) | (+0.71, +1.01) | (+0.37, +0.86) | (+0.40, +0.33) | (+0.98, +0.33) |
| quart 3D tang 5 | (+1.05, +0.86) | (+0.73, +1.03) | (+0.39, +0.88) | (+0.42, +0.35) | (+1.00, +0.35) |

Antes de decolar, confira a posição de cada drone:

```bash
ros2 topic echo --once /cf_0/state
```

#### Rodar nos drones reais

1. Deixe exatamente N drones `enabled` no `config/crazyflies_real.yaml`, com nomes `cf_0 .. cf_{N-1}` (o `sph` lê o N de lá).
2. Coloque os drones nas posições da tabela.
3. Rode:

```bash
./real_tmux.sh up
./real_tmux.sh takeoff                                 # o controlador só age com todos acima de 0,4 m
#Linha do teste
./real_tmux.sh land                                    # ou Ctrl+b l dentro do tmux
./real_tmux.sh stop
```
Logs do SPH: `src/logs/runs/<data>_<RUN_LABEL>/`.
# ---------- CrazySim ----------
# 3 drones: circular
N_DRONES=3 SPH_MASS=auto SPH_GAIN=0.5 POT_K=10.0 POT_R=0.346 SPH_GAMMA=3.0 SPH_SPACING=0.6 SPH_H=0.5 SPH_ZETTA=50.0 RUN_LABEL=cs_circ_3 ./dev_tmux.sh up
# 4 drones: quartico 2D / 3D
N_DRONES=4 SPH_MASS=auto SPH_GAIN=0.5 POTENTIAL=quartic POT_K=60.0 POT_R=0.0142 SPH_GAMMA=3.0 SPH_SPACING=0.6 SPH_H=0.5 SPH_ZETTA=50.0 RUN_LABEL=cs_quart2d_4 ./dev_tmux.sh up

N_DRONES=4 SPH_GAIN=0.5 SPH_GAMMA=3.0 SPH_DIM=3 POTENTIAL=quartic POT_R=0.0142 POT_XC=0.7 POT_YC=0.7 POT_K=60.0 SPH_MASS=330 SPH_H=0.5 SPH_SPACING=0.5 Z_MIN=0.2 Z_MARGIN=0.1 Z_MAX=1.5 Z_WALL_K=8 RUN_LABEL=cs_quart3d_4 ./dev_tmux.sh up

N_DRONES=4 SPH_MASS=auto SPH_GAIN=0.5 POTENTIAL=quartic POT_K=60.0 POT_KT=5.0 POT_R=0.0142 SPH_GAMMA=3.0 SPH_SPACING=0.6 SPH_H=0.5 SPH_ZETTA=50.0 RUN_LABEL=cs_quart2d_4 ./dev_tmux.sh up

N_DRONES=4 SPH_GAIN=0.5 SPH_GAMMA=3.0 SPH_DIM=3 POTENTIAL=quartic POT_R=0.0142 POT_XC=0.7 POT_YC=0.7 POT_K=60.0 POT_KT=2.0 SPH_MASS=330 SPH_H=0.5 SPH_SPACING=0.5 Z_MIN=0.2 Z_MARGIN=0.1 Z_MAX=1.5 Z_WALL_K=8 RUN_LABEL=cs_quart3d_4 ./dev_tmux.sh up


# 5 drones: quartico 2D / 3D / 2D tangente / 3D tangente
N_DRONES=5 SPH_MASS=auto SPH_GAIN=0.5 POTENTIAL=quartic POT_K=20.0 POT_R=0.035 SPH_GAMMA=3.0 SPH_SPACING=0.6 SPH_H=0.5 SPH_ZETTA=50.0 RUN_LABEL=cs_quart2d_5 ./dev_tmux.sh up
N_DRONES=5 SPH_GAIN=0.5 SPH_GAMMA=3.0 SPH_DIM=3 POTENTIAL=quartic POT_R=0.035 POT_XC=0.7 POT_YC=0.7 POT_K=60.0 SPH_MASS=330 SPH_H=0.5 SPH_SPACING=0.5 Z_MIN=0.2 Z_MARGIN=0.1 Z_MAX=1.5 Z_WALL_K=8 RUN_LABEL=cs_quart3d_5 ./dev_tmux.sh up
N_DRONES=5 SPH_MASS=auto SPH_GAIN=0.5 POTENTIAL=quartic POT_K=20.0 POT_R=0.035 SPH_GAMMA=3.0 SPH_SPACING=0.6 SPH_H=0.5 SPH_ZETTA=50.0 POT_KT=3.0 RUN_LABEL=cs_quart2d_tang_5 ./dev_tmux.sh up
N_DRONES=5 SPH_GAIN=0.5 SPH_GAMMA=3.0 SPH_DIM=3 POTENTIAL=quartic POT_R=0.035 POT_XC=0.72 POT_YC=0.72 POT_K=60.0 SPH_MASS=330 SPH_H=0.5 SPH_SPACING=0.5 POT_KT=1.0 Z_MIN=0.2 Z_MARGIN=0.1 Z_MAX=1.5 Z_WALL_K=8 RUN_LABEL=cs_quart3d_tang_5 ./dev_tmux.sh up

# ---------- Reais ----------
# 3 drones: circular
SPH_MASS=auto SPH_GAIN=0.5 POT_K=10.0 POT_R=0.346 SPH_GAMMA=3.0 SPH_SPACING=0.6 SPH_H=0.5 SPH_ZETTA=50.0 RUN_LABEL=real_circ_3 ./real_tmux.sh sph
# 4 drones: quartico 2D / 3D
SPH_MASS=auto SPH_GAIN=0.5 POTENTIAL=quartic POT_K=60.0 POT_R=0.0142 SPH_GAMMA=3.0 SPH_SPACING=0.6 SPH_H=0.5 SPH_ZETTA=50.0 RUN_LABEL=real_quart2d_4 ./real_tmux.sh sph
SPH_GAIN=0.5 SPH_GAMMA=3.0 SPH_DIM=3 POTENTIAL=quartic POT_R=0.0142 POT_XC=0.7 POT_YC=0.7 POT_K=60.0 SPH_MASS=330 SPH_H=0.5 SPH_SPACING=0.5 Z_MIN=0.2 Z_MARGIN=0.1 Z_MAX=1.5 Z_WALL_K=8 RUN_LABEL=real_quart3d_4 ./real_tmux.sh sph
# 5 drones: quartico 2D / 3D / 2D tangente / 3D tangente
SPH_MASS=auto SPH_GAIN=0.5 POTENTIAL=quartic POT_K=20.0 POT_R=0.035 SPH_GAMMA=3.0 SPH_SPACING=0.6 SPH_H=0.5 SPH_ZETTA=50.0 RUN_LABEL=real_quart2d_5 ./real_tmux.sh sph
SPH_GAIN=0.5 SPH_GAMMA=3.0 SPH_DIM=3 POTENTIAL=quartic POT_R=0.035 POT_XC=0.7 POT_YC=0.7 POT_K=60.0 SPH_MASS=330 SPH_H=0.5 SPH_SPACING=0.5 Z_MIN=0.2 Z_MARGIN=0.1 Z_MAX=1.5 Z_WALL_K=8 RUN_LABEL=real_quart3d_5 ./real_tmux.sh sph
SPH_MASS=auto SPH_GAIN=0.5 POTENTIAL=quartic POT_K=20.0 POT_R=0.035 SPH_GAMMA=3.0 SPH_SPACING=0.6 SPH_H=0.5 SPH_ZETTA=50.0 POT_KT=3.0 RUN_LABEL=real_quart2d_tang_5 ./real_tmux.sh sph
SPH_GAIN=0.5 SPH_GAMMA=3.0 SPH_DIM=3 POTENTIAL=quartic POT_R=0.035 POT_XC=0.72 POT_YC=0.72 POT_K=60.0 SPH_MASS=330 SPH_H=0.5 SPH_SPACING=0.5 POT_KT=1.0 Z_MIN=0.2 Z_MARGIN=0.1 Z_MAX=1.5 Z_WALL_K=8 RUN_LABEL=real_quart3d_tang_5 ./real_tmux.sh sph

## Testes de movimento

Os scripts ficam em `src/drone_controller/testes/` e rodam sem o controlador SPH.
Eles decolam pelo serviço `/cf_i/takeoff`, se movem com `Commander.send_velocity_world()` e pousam com `Commander.land()`.

| Script | Movimento |
|---|---|
| `frente_tras.py` | +x, pausa, −x, pausa |
| `frente_tras_lados.py` | frente (+x), trás (−x), lado (+y), outro lado (−y, 2× o tempo) e volta ao centro (+y) |

Argumentos principais:

| Argumento | O que faz | Padrão |
|---|---|---|
| `--v` | velocidade (m/s) | 0.2 |
| `--tempo` | duração de cada movimento (s) | 3 |
| `--pausa` | tempo parado entre os movimentos (s) | 2 |
| `--repeticoes` | quantas vezes repete a sequência | 1 |
| `--height` | altura de decolagem (m) | 0.5 |

O deslocamento de cada movimento é `v × tempo`. A lista completa de argumentos sai com `--help`.

**Segurança:** se um drone sai da cerca (±1,5 m) ou passa 1 s sem `/state`, todos recebem 0 m/s e pousam.
Ctrl+C também pousa.
Logs: `src/logs/testes/`.

### CrazySim

```bash
./dev_tmux.sh test                                        # lista os testes
./dev_tmux.sh test frente_tras --v 0.2 --tempo 3
./dev_tmux.sh test frente_tras_lados --v 0.2 --tempo 3
./dev_tmux.sh stop                                        # ao terminar
```

A primeira chamada sobe o CrazySim sem o controlador SPH e leva 1 a 2 min. As chamadas seguintes reaproveitam a simulação.
Para usar outro número de drones, passe `N_DRONES=3` antes de `./dev_tmux.sh test` na primeira chamada.

### Drones reais

```bash
./real_tmux.sh up                                         # sobe o Crazyswarm2 (config/crazyflies_real.yaml)
# em outro terminal, ou Ctrl+b d para sair do tmux:
./real_tmux.sh test frente_tras --v 0.2 --tempo 2
./real_tmux.sh test frente_tras_lados --v 0.2 --tempo 2
./real_tmux.sh stop
```

Os drones habilitados no yaml precisam se chamar `cf_0` .. `cf_{N-1}`.
Comece com 1 drone e `--tempo 2`, que dá 0,4 m por movimento.
O comando se recusa a rodar se o controlador SPH (janela `ctrl`) estiver aberto.

cd ~/SPH_Project
R=src/logs/runs
P="python3 src/plot_plain_sph.py"

# ================= CONFORMAÇÃO =================
# dinâmica simplificada
$P $R/20261020_100406_p_circ_20          --no-open   # circular, 20
$P $R/20261017_122551_p_esfera           --no-open   # esférico, 8
$P $R/20261020_100713_p_quart2d          --no-open   # quártico 2D, 12
$P $R/20261020_101717_p_quart3d          --no-open   # quártico 3D, 8
# CrazySim
$P $R/20261020_105555_cs_circ_8          --no-open   # circular, 8
$P $R/20261020_111550_cs_esfera_8        --no-open   # esférico, 8
$P $R/20261020_110216_cs_quart2d_6       --no-open   # quártico 2D, 6
$P $R/20261020_112622_cs_quart3d_8       --no-open   # quártico 3D, 8

# ================= CIRCULAÇÃO =================
# dinâmica simplificada
$P $R/20261020_104400_p_quart2d_tang     --no-open   # quártico 2D, 12
$P $R/20261020_104608_p_quart3d_tang     --no-open   # quártico 3D, 8
# CrazySim
$P $R/20261020_114106_cs_quart2d_tang_6  --no-open   # quártico 2D, 6
$P $R/20261020_114405_cs_quart3d_tang_8  --no-open   # quártico 3D, 8

# ================= OBSTÁCULOS =================
# dinâmica simplificada
$P $R/20261022_004115_p_obs_circ_portao_6   --no-open   # passagem estreitada, 6
$P $R/20261022_004442_p_obs_esfera_6        --no-open   # esfera, 6
$P $R/20261022_005144_p_obs_quart3d_6       --no-open   # curva espacial, 6
$P $R/20261022_010617_p_obs_quart2d_tang_6  --no-open   # curva com circulação, 6
# CrazySim
$P $R/20261022_004254_cs_obs_circ_portao_3  --no-open   # passagem estreitada, 3
$P $R/20261022_004859_cs_obs_esfera_3       --no-open   # esfera, 3
$P $R/20261022_010954_cs_obs_quart3d_3      --no-open   # curva espacial, 3
$P $R/20261022_011206_cs_obs_quart2d_tang_6 --no-open   # curva com circulação, 3 (label diz 6)

## Rodar sem tmux

Cada passo roda num terminal próprio, a partir de `~/SPH_Project`.
Em todo terminal que entra no container, carregue o ROS primeiro:

```bash
docker compose exec crazysim bash
source /opt/ros/humble/setup.bash
source /CrazySim/crazyswarm2_ws/install/setup.bash
source /root/dev_ws/src/dev_setup.bash
```

### CrazySim

**0. Host: gerar a configuração e reiniciar o container**
```bash
python3 gen_config.py --n 3 --radius 0.35 --config-dir config
docker compose up -d crazysim && docker compose restart -t 1 crazysim
```

**1. Terminal 1: Gazebo e firmware**
```bash
rm -rf /tmp/gz* /tmp/sitl*
cd /CrazySim/crazyflie-firmware
bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_multiagent_headless.sh -f agents.txt
```

**2. Terminal 2: Crazyswarm2** (quando os drones aparecerem no terminal 1)
```bash
ros2 launch crazyflie launch.py backend:=cflib rviz:=False
```

**3. Terminal 3: teste de movimento** (decola e pousa sozinho)
```bash
cd /root/dev_ws/src/drone_controller/testes
python3 frente_tras.py --v 0.2 --tempo 3
python3 frente_tras_lados.py --v 0.2 --tempo 3
```

**3'. Terminal 3: teste SPH**

No host, gere os argumentos do launch com a linha do teste, trocando o final por `./dev_tmux.sh args`:
```bash
ARGS=$(N_DRONES=3 SPH_DIM=2 POTENTIAL=circle POT_R=0.346 POT_K=1.0 POT_BETA=0 POT_KT=0 SPH_GAIN=0.3 SPH_H=0.4529 SPH_MASS=437.12 SPH_ZETTA=30 STALE_POLICY=hold STATE_TIMEOUT_S=1.0 RUN_LABEL=sim_circ_3 ./dev_tmux.sh args)
echo "$ARGS"
```
Depois decole e suba o controlador:
```bash
docker compose exec crazysim bash -lc "source /opt/ros/humble/setup.bash && source /CrazySim/crazyswarm2_ws/install/setup.bash && ros2 service call /all/takeoff crazyflie_interfaces/srv/Takeoff '{height: 0.5, duration: {sec: 2, nanosec: 0}}'"
docker compose exec crazysim bash -lc "source /opt/ros/humble/setup.bash && source /CrazySim/crazyswarm2_ws/install/setup.bash && source /root/dev_ws/src/dev_setup.bash && ros2 launch drone_controller plain_sph.launch.py $ARGS"
```

**4. Pousar e encerrar**
```bash
ros2 topic pub -w 1 -t 5 /sph/land std_msgs/msg/Empty '{}'      # dentro do container: pousa pelo controlador
```
Depois dê Ctrl+C no terminal 3 (grava os CSVs), depois no 2 e no 1, e reinicie o container:
```bash
docker compose restart -t 1 crazysim
```

### Drones reais

**0. Host: usar o yaml dos drones reais**
```bash
cp config/crazyflies_real.yaml config/crazyflies.yaml
docker compose up -d crazysim && docker compose restart -t 1 crazysim
```

**1. Terminal 1: Crazyswarm2 pelo Crazyradio**
```bash
ros2 launch crazyflie launch.py backend:=cflib rviz:=False
```

**2. Terminal 2: teste de movimento** (decola e pousa sozinho)
```bash
cd /root/dev_ws/src/drone_controller/testes
python3 frente_tras.py --v 0.2 --tempo 2
```

**2'. Terminal 2: teste SPH**

No host, gere os argumentos com os parâmetros do teste e os de segurança do real:
```bash
ARGS=$(N_DRONES=3 PLATFORM=real SIM_CLOCK=wall START_GATE=flight STALE_POLICY=hold STATE_TIMEOUT_S=1.0 FIXED_Z=0.5 MAX_VELOCITY=0.2 MAX_VZ=0.2 FENCE_XMIN=-1.5 FENCE_XMAX=1.5 FENCE_YMIN=-1.5 FENCE_YMAX=1.5 \
  SPH_DIM=2 POTENTIAL=circle POT_R=0.346 POT_K=1.0 POT_BETA=0 POT_KT=0 SPH_GAIN=0.3 SPH_H=0.4529 SPH_MASS=437.12 SPH_ZETTA=30 RUN_LABEL=real_circ_3 ./dev_tmux.sh args)
```
Depois decole e suba o controlador:
```bash
docker compose exec crazysim bash -lc "source /opt/ros/humble/setup.bash && source /CrazySim/crazyswarm2_ws/install/setup.bash && ros2 service call /all/takeoff crazyflie_interfaces/srv/Takeoff '{height: 0.5, duration: {sec: 3, nanosec: 0}}'"
docker compose exec crazysim bash -lc "source /opt/ros/humble/setup.bash && source /CrazySim/crazyswarm2_ws/install/setup.bash && source /root/dev_ws/src/dev_setup.bash && ros2 launch drone_controller plain_sph.launch.py $ARGS"
```

**3. Pousar**

Dentro do container, pouse pelo controlador:
```bash
ros2 topic pub -w 1 -t 5 /sph/land std_msgs/msg/Empty '{}'
```
Pouso de emergência pelo firmware, que funciona mesmo com o controlador travado:
```bash
ros2 service call /all/land crazyflie_interfaces/srv/Land "{height: 0.0, duration: {sec: 3, nanosec: 0}}"
```
Depois Ctrl+C no controlador e no Crazyswarm2.