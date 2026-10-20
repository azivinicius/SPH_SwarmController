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
### Testes da cadeia de comando (CrazySim e drones reais)

Valida o voo e o seu pacote (subscribers → `sph_core` → Commander) com SPH e potencial desligados.
Faça na ordem da tabela: os testes 1–2 usam só o firmware e o Crazyswarm2; do 3 em diante entra o `drone_controller`.

Base para os testes do controlador no CrazySim (rode uma vez no terminal):
```bash
export N_DRONES=4 SPH_GAIN=0 POT_K=0 STALE_POLICY=hold AUTO_TAKEOFF=1 SPH_MASS=100.0
```
Nos drones reais, `./real_tmux.sh ctrl` já fixa esses parâmetros (amplitude 0,2 m/s, período `SQ_STEP`).

| # | Teste | O que valida | CrazySim (`dev_tmux.sh`) | Real (`real_tmux.sh`) |
|---|---|---|---|---|
| 1 | Decolar e pousar | Rádio, estimador, `/all/takeoff` e `/all/land` | `CTRL=0 ./dev_tmux.sh up` → `./real_tmux.sh takeoff` → `./real_tmux.sh land` | `up` → `takeoff` → `land` |
| 2 | Quadrado com `go_to` | High-level commander e posição no mundo, sem o seu pacote | igual ao 1, com `SQ_N=4 ./real_tmux.sh square` | `SQ_N=1..4 square` |
| 3 | `zero` | Cadeia completa sem movimento; deriva em pairado | `TEST_PROFILE=zero ./dev_tmux.sh up` | `TEST=zero ctrl` |
| 4 | `step` | Resposta ao degrau: ganho, atraso, sobressinal | `TEST_PROFILE=step TEST_AMP=0.2 TEST_PERIOD=4 TEST_AXIS=x ./dev_tmux.sh up` | `TEST=step ctrl` |
| 5 | `square` | Degraus alternados ±amp: reversão de sentido | `TEST_PROFILE=square TEST_AMP=0.2 TEST_PERIOD=6 ./dev_tmux.sh up` | `TEST=square ctrl` |
| 6 | `sine` | Seguimento de referência suave: fase e amplitude | `TEST_PROFILE=sine TEST_AMP=0.2 TEST_PERIOD=8 ./dev_tmux.sh up` | `TEST=sine ctrl` |
| 7 | `chirp` | Resposta em frequência (0,05 a 1,5 Hz) | `TEST_PROFILE=chirp TEST_AMP=0.15 TEST_DURATION=60 ./dev_tmux.sh up` | `TEST=chirp ctrl` |
| 8 | `box` | Quadrado gerado pelo seu controlador: malha de posição, saturação de 0,2 m/s, sentido anti-horário | `TEST_PROFILE=box TEST_AMP=0.2 TEST_PERIOD=6 TEST_DELAY=6 TEST_DURATION=40 TEST_SIDE=1.0 POT_XC=0 POT_YC=0 RUN_LABEL=cs_box ./dev_tmux.sh up` | `TEST=box ctrl` |

Notas:
- `CTRL=0` sobe o CrazySim e o Crazyswarm2 sem a janela `ctrl`; encerre com `./dev_tmux.sh stop`, não com o `stop` do `real_tmux.sh`.
- Os testes 4–7 mandam o mesmo comando para todos os drones, que se deslocam em bloco ao longo de `TEST_AXIS`; nos reais, confira a cerca (±1,5 m) e o espaço livre.
- O teste 8 usa no máximo 4 drones (um por vértice). O `TEST_PERIOD` deve ser pelo menos `TEST_SIDE`/0,2 s mais uma folga.
- Drones reais: comece com 1 drone e suba até 4. Sequência: `up` → `takeoff` → `square` ou `ctrl` → `land` (ou `Ctrl+b l`) → `stop`.
- Depois de cada ensaio: `./dev_tmux.sh analyze`. Em `tick_*.csv`, `clipped` indica quando a saturação agiu, e `rx_since`/`state_age_ms` indicam atraso de mensagens.

export SPAWN_RADIUS=1.0 SPH_MASS=auto SPH_H=0.5 SPH_SPACING=0.6 SPH_GAIN=0.5 SPH_GAMMA=3.0 SPH_ZETTA=50.0 \
       POT_XC=0 POT_YC=0 FENCE_XMIN=-1.5 FENCE_XMAX=1.5 FENCE_YMIN=-1.5 FENCE_YMAX=1.5 \
       HL_HEIGHT=0.8 FIXED_Z=0.8 TAKEOFF_HEIGHT=0.8 Z_MAX=1.0
# reais: ./real_tmux.sh up -> takeoff -> (comando) -> land -> stop
# no CrazySim: troque "./real_tmux.sh ctrl" por "N_DRONES=<N> ./dev_tmux.sh up" (sem o TEST=)

# ---------- 3 drones ----------
SQ_N=3 ./real_tmux.sh square
TEST=box ./real_tmux.sh ctrl
POT_K=10.0 POT_R=0.346 RUN_LABEL=real_circ_3 ./real_tmux.sh ctrl

# ---------- 4 drones ----------
SQ_N=4 ./real_tmux.sh square
TEST=box ./real_tmux.sh ctrl
POT_K=10.0 POT_R=0.424 RUN_LABEL=real_circ_4 ./real_tmux.sh ctrl
POTENTIAL=quartic POT_K=60.0 POT_R=0.0142 RUN_LABEL=real_quart2d_4 ./real_tmux.sh ctrl
POTENTIAL=quartic POT_K=60.0 POT_R=0.0142 POT_KT=6.0 RUN_LABEL=real_quart2d_tang_4 ./real_tmux.sh ctrl

# ---------- 5 drones ----------
POT_K=10.0 POT_R=0.51 RUN_LABEL=real_circ_5 ./real_tmux.sh ctrl
POTENTIAL=quartic POT_K=20.0 POT_R=0.035 RUN_LABEL=real_quart2d_5 ./real_tmux.sh ctrl
POTENTIAL=quartic POT_K=20.0 POT_R=0.035 POT_KT=3.0 RUN_LABEL=real_quart2d_tang_5 ./real_tmux.sh ctrl