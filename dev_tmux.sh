#!/usr/bin/env bash
# dev_tmux.sh -- run the whole SPH_Project stack from ONE host terminal, driven by swarm.conf.
# Run on the HOST from ~/SPH_Project.
#
#   ./dev_tmux.sh up      generate configs for N_DRONES, start container + tmux session, attach
#   ./dev_tmux.sh up-particles  controller + Gazebo particles, NO physics / firmware (see up_particles)
#   ./dev_tmux.sh up-swarm  same as `up`, but runs ALL controllers in ONE process (swarm_sph.launch.py)
#   ./dev_tmux.sh attach  re-attach to the running session
#   ./dev_tmux.sh stop    Ctrl+C the controller (flushes CSVs), stop everything, fix log ownership.
#                         Also bound to  Ctrl+b K  inside the session (works from any window).
#   ./dev_tmux.sh show    print the resolved config and the launch command, then exit
#   ./dev_tmux.sh analyze [run_id]   analyse a run (default: the last one) -> logs/runs/<id>/analysis/summary.txt
#   ./dev_tmux.sh check   compare host config files with what the container actually sees,
#                         and count running sim processes (use this when a run looks wrong)
#
# Every run gets its own folder  src/logs/runs/<timestamp>_<RUN_LABEL>/  (tick/rx CSVs, events.log,
# rtf.csv, meta.json). Label a run with:  RUN_LABEL=hover ./dev_tmux.sh up-swarm
# DETACH=1 (or ATTACH=0) starts everything without attaching to tmux.
#
# Per-run overrides:  N_DRONES=5 POT_R=0.8 ./dev_tmux.sh up
# Other config file:  CONF=configs/five.conf ./dev_tmux.sh up

SESSION="${SESSION:-sph}"
CONF="${CONF:-swarm.conf}"
ATTACH="${ATTACH:-1}"        # 0 = start the session but do not attach (scripts/tests)
[ "${DETACH:-0}" = "1" ] && ATTACH=0

cd "$(dirname "$0")" || exit 1
[ -f "$CONF" ] || { echo "config file '$CONF' not found"; exit 1; }
# shellcheck disable=SC1090
source "$CONF"

DC="docker compose exec crazysim bash -lc"
# 'bash -lc' is non-interactive, so it does NOT read /root/.bashrc, which is where the
# Dockerfile sources ROS. Source it explicitly in every command that needs ros2.
ROS_ENV="source /opt/ros/humble/setup.bash && source /CrazySim/crazyswarm2_ws/install/setup.bash"

# Where the container sees the two generated config files (see docker-compose.yaml).
# The third path is what `ros2 launch crazyflie launch.py` really loads (colcon install space,
# a symlink into the bind-mounted src file when built with --symlink-install).
C_AGENTS=/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/launch/drone_spawn_list/agents.txt
C_YAML_SRC=/CrazySim/crazyswarm2_ws/src/crazyswarm2/crazyflie/config/crazyflies.yaml
C_YAML_INST=/CrazySim/crazyswarm2_ws/install/crazyflie/share/crazyflie/config/crazyflies.yaml

host_sum() { sha256sum "$1" | cut -d' ' -f1; }
cont_sum() { docker compose exec -T crazysim sh -c "sha256sum '$1' 2>/dev/null" | cut -d' ' -f1; }

# 0 = the container sees exactly the files on the host.
# Single-file bind mounts pin an INODE: if git (checkout/pull/stash) or an editor replaced the
# host file after the container started, the container keeps the OLD content while the host
# shows the new one. Only restarting the container re-resolves the mount.
mounts_ok() {
  local rc=0 pair h c cs
  for pair in "config/agents.txt|$C_AGENTS" \
              "config/crazyflies.yaml|$C_YAML_SRC" \
              "config/crazyflies.yaml|$C_YAML_INST"; do
    h=${pair%%|*}; c=${pair#*|}
    cs=$(cont_sum "$c")
    [ -z "$cs" ] && continue                      # path absent in this image: nothing to compare
    if [ "$cs" != "$(host_sum "$h")" ]; then
      echo "  MISMATCH: container's $c differs from host $h"
      rc=1
    fi
  done
  return $rc
}

# Start from a clean slate every time: restarting kills every leftover process from earlier runs
# (gz, cf2, crazyflie_server, rviz2, ros2 daemon...) and re-resolves the bind mounts.
fresh_container() {
  docker compose up -d crazysim || exit 1
  echo "restarting container: clean process space + fresh bind mounts..."
  docker compose restart -t 1 crazysim >/dev/null || exit 1
  if ! mounts_ok; then
    echo "bind mounts still stale after restart -> recreating the container (colcon build will rerun)"
    docker compose up -d --force-recreate crazysim || exit 1
    mounts_ok || { echo "ERROR: container still does not see the generated config. Aborting."; exit 1; }
  fi
}

# Resolve 'auto' radii and build the launch args. Called by `up` and `show`.
resolve() {
  # 3D mode (SPH_DIM=3): sphere potential + 3D SPH. 'auto' mass / radius are solved for a sphere:
  # POT_R=auto -> the radius at which N drones sit SPH_SPACING apart, SPH_MASS=auto -> density rho0 there.
  SPH_DIM="${SPH_DIM:-2}"
  if [ "$SPH_DIM" = "3" ]; then
    if [ "$SPH_MASS" = "auto" ]; then
      SPH_MASS=$(python3 gen_config.py --n "$N_DRONES" --dim 3 --mass auto --radius "$POT_R" --spacing "$SPH_SPACING" \
                 --h "$SPH_H" --rho0 "$SPH_RHO0" --print-mass) || exit 1
    fi
    if [ "$POT_R" = "auto" ]; then
      POT_R=$(python3 gen_config.py --n "$N_DRONES" --dim 3 --mass auto --radius auto --spacing "$SPH_SPACING" \
              --h "$SPH_H" --rho0 "$SPH_RHO0" --print-radius) || exit 1
    fi
  fi
  # SPH_MASS=auto: solve the mass (and the ring radius) from SPH_SPACING, SPH_H and SPH_RHO0
  if [ "$SPH_MASS" = "auto" ]; then
    SPH_MASS=$(python3 gen_config.py --n "$N_DRONES" --mass auto --spacing "$SPH_SPACING" \
               --h "$SPH_H" --rho0 "$SPH_RHO0" --print-mass) || exit 1
    if [ "$SPAWN_RADIUS" = "auto" ]; then
      SPAWN_RADIUS=$(python3 gen_config.py --n "$N_DRONES" --mass auto --spacing "$SPH_SPACING" \
                     --h "$SPH_H" --rho0 "$SPH_RHO0" --print-radius) || exit 1
    fi
  fi
  if [ "$SPAWN_RADIUS" = "auto" ]; then
    SPAWN_RADIUS=$(python3 gen_config.py --n "$N_DRONES" --radius auto --mass "$SPH_MASS" \
                   --h "$SPH_H" --rho0 "$SPH_RHO0" --min-spacing "$MIN_SPACING" --print-radius) || exit 1
  fi
  [ "$POT_R" = "auto" ] && POT_R="$SPAWN_RADIUS"
  # diagnostics / test switches (defaults reproduce the old behaviour; see swarm.conf)
  RUN_LABEL="${RUN_LABEL:-run}"
  RUN_ID="${RUN_ID:-$(date +%Y%m%d_%H%M%S)_${RUN_LABEL}}"
  GIT_REV="$(git rev-parse --short HEAD 2>/dev/null || echo unknown)"
  [ -n "$(git status --porcelain 2>/dev/null)" ] && GIT_REV="${GIT_REV}-dirty"
  # every SPH / potential knob from swarm.conf goes to the controller (both launch files accept them)
  LAUNCH_ARGS="total_robots:=$N_DRONES fixed_z:=$FIXED_Z particle_mass:=$SPH_MASS smoothing_h:=$SPH_H"
  LAUNCH_ARGS="$LAUNCH_ARGS rho_0:=$SPH_RHO0 gamma:=$SPH_GAMMA alpha:=$SPH_ALPHA beta:=$SPH_BETA"
  LAUNCH_ARGS="$LAUNCH_ARGS zetta_extra:=$SPH_ZETTA xc:=$POT_XC yc:=$POT_YC R:=$POT_R f_k:=$POT_K f_beta:=$POT_BETA"
  LAUNCH_ARGS="$LAUNCH_ARGS sph_gain:=${SPH_GAIN:-1.0} control_period:=${CONTROL_PERIOD:-0.05} dt_model:=${DT_MODEL:-0.0}"
  LAUNCH_ARGS="$LAUNCH_ARGS stale_policy:=${STALE_POLICY:-predict} start_gate:=${START_GATE:-none}"
  LAUNCH_ARGS="$LAUNCH_ARGS test_profile:=${TEST_PROFILE:-none} test_amp:=${TEST_AMP:-0.15} test_period:=${TEST_PERIOD:-4.0}"
  LAUNCH_ARGS="$LAUNCH_ARGS test_axis:=${TEST_AXIS:-x} test_delay:=${TEST_DELAY:-3.0} test_duration:=${TEST_DURATION:-60.0}"
  LAUNCH_ARGS="$LAUNCH_ARGS sim_clock:=${SIM_CLOCK:-wall} max_velocity:=${MAX_VELOCITY:-0.5} platform:=${PLATFORM:-sim}"
  LAUNCH_ARGS="$LAUNCH_ARGS fence_xmin:=${FENCE_XMIN:-nan} fence_xmax:=${FENCE_XMAX:-nan} fence_ymin:=${FENCE_YMIN:-nan} fence_ymax:=${FENCE_YMAX:-nan}"
  LAUNCH_ARGS="$LAUNCH_ARGS state_timeout_s:=${STATE_TIMEOUT_S:-0}"
  LAUNCH_ARGS="$LAUNCH_ARGS dim:=$SPH_DIM zc:=${POT_ZC:-1.0} max_vz:=${MAX_VZ:-0.25} z_min:=${Z_MIN:-0.25} z_max:=${Z_MAX:-2.5}"
  LAUNCH_ARGS="$LAUNCH_ARGS z_margin:=${Z_MARGIN:-0.3} z_wall_k:=${Z_WALL_K:-2.0} z_stretch:=${Z_STRETCH:-1.0}"
  # target shape: POTENTIAL=default|circle|sphere|quartic ; with quartic, POT_K scales the force (2 = original, 0.5 = 4x weaker)
  LAUNCH_ARGS="$LAUNCH_ARGS potential:=${POTENTIAL:-default}"
  LAUNCH_ARGS="$LAUNCH_ARGS obstacles:=${OBSTACLES:-none} obs_mass:=${OBS_MASS:-10.0}"
  LAUNCH_ARGS="$LAUNCH_ARGS run_id:=$RUN_ID run_label:=$RUN_LABEL git_rev:=$GIT_REV"
}
# which launch file the ctrl window runs; `up-swarm` switches it to the single-process controller
CTRL_LAUNCH="${CTRL_LAUNCH:-plain_sph.launch.py}"

show() {
  resolve
  echo "config file : $CONF"
  echo "drones      : $N_DRONES (spawn radius $SPAWN_RADIUS m)   dim: $SPH_DIM   mass: $SPH_MASS   potential R: $POT_R"
  [ "$SPH_DIM" = "3" ] && echo "3D sphere   : centre ($POT_XC, $POT_YC, ${POT_ZC:-1.0}), z limits ${Z_MIN:-0.25}..${Z_MAX:-2.5} m, max_vz ${MAX_VZ:-0.25} m/s, z_stretch ${Z_STRETCH:-1.0}"
  echo "launch file : $CTRL_LAUNCH"
  echo "launch args : $LAUNCH_ARGS"
  echo "run id      : $RUN_ID   (logs: src/logs/runs/$RUN_ID)"
}

up() {
  resolve
  # Refuse to touch anything if a session is already running. Re-attaching to it would silently
  # keep the OLD drone count while the config files on disk had already been rewritten.
  if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "A '$SESSION' tmux session is already running (it may use a different N_DRONES)."
    echo "  attach to it : $0 attach"
    echo "  start fresh  : $0 stop && N_DRONES=$N_DRONES $0 up"
    exit 1
  fi

  python3 gen_config.py --n "$N_DRONES" --radius "$SPAWN_RADIUS" --config-dir config || exit 1
  fresh_container
  echo "$RUN_ID" > .last_run_id
  echo "run id: $RUN_ID  -> src/logs/runs/$RUN_ID   (analyse with: ./dev_tmux.sh analyze)"
  xhost +local:root >/dev/null 2>&1

  # Remove logs of drones that no longer exist (e.g. cf_3/cf_4 left over from a 5-drone run
  # when starting 3). Done inside the container because the files may be owned by root.
  CLEAN_CMD='cd /root/dev_ws/src/logs 2>/dev/null || exit 0
    for f in plain_sph_cf_*.csv; do
      [ -e "$f" ] || continue
      id=${f#plain_sph_cf_}; id=${id%.csv}
      case "$id" in (*[!0-9]*|"") continue;; esac
      if [ "$id" -ge '"$N_DRONES"' ]; then rm -f "$f"; echo "removed stale log $f"; fi
    done; true'
  docker compose exec -T crazysim bash -lc "$CLEAN_CMD"

  tmux new-session -d -s "$SESSION" -n sim
  tmux send-keys -t "$SESSION:sim" \
    "$DC 'echo \"[dev_tmux] N_DRONES=$N_DRONES -- agents.txt as the container sees it:\"; cat $C_AGENTS; echo; rm -rf /tmp/gz* /tmp/sitl* && cd /CrazySim/crazyflie-firmware && bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_multiagent_headless.sh -f agents.txt'" C-m

  tmux new-window -t "$SESSION" -n swarm
  tmux send-keys -t "$SESSION:swarm" \
    "sleep $SIM_WAIT; $DC '$ROS_ENV && ros2 launch crazyflie launch.py backend:=cflib rviz:=$RVIZ'" C-m

  # GZ_GUI=1: Gazebo window on the (headless) CrazySim server (default 0 here). Costs CPU: keep the RTF >= 0.9
  if [ "${GZ_GUI:-0}" = "1" ]; then
    tmux new-window -t "$SESSION" -n gui
    tmux send-keys -t "$SESSION:gui" \
      "sleep $SIM_WAIT; $DC 'cd /CrazySim/crazyflie-firmware && source tools/crazyflie-simulation/simulator_files/gazebo/launch/setup_gz.bash \$PWD \$PWD/sitl_make/build && gz sim -g'" C-m
  fi
  # dev_setup.bash, NOT install/setup.bash (README known issue #1)
  tmux new-window -t "$SESSION" -n ctrl
  tmux send-keys -t "$SESSION:ctrl" \
    "sleep $((SIM_WAIT + SWARM_WAIT)); $DC '$ROS_ENV && cd /root/dev_ws && colcon build --symlink-install && source src/dev_setup.bash && ros2 launch drone_controller $CTRL_LAUNCH $LAUNCH_ARGS'" C-m

  # Gazebo real-time factor -> logs/runs/<id>/rtf.csv
  tmux new-window -t "$SESSION" -n rtf
  tmux send-keys -t "$SESSION:rtf" \
    "sleep $SIM_WAIT; $DC '$ROS_ENV && python3 /root/dev_ws/src/tools/rtf_logger.py --out /root/dev_ws/src/logs/runs/$RUN_ID/rtf.csv'" C-m

  # CPU / memory / clock / temperature per process group -> logs/runs/<id>/sys.csv
  tmux new-window -t "$SESSION" -n sys
  tmux send-keys -t "$SESSION:sys" \
    "$DC 'python3 /root/dev_ws/src/tools/sys_monitor.py --out /root/dev_ws/src/logs/runs/$RUN_ID/sys.csv'" C-m

  tmux new-window -t "$SESSION" -n shell
  if [ "$AUTO_TAKEOFF" = "1" ]; then
    tmux send-keys -t "$SESSION:shell" \
      "sleep $((SIM_WAIT + SWARM_WAIT + TAKEOFF_WAIT)); $DC '$ROS_ENV && ros2 service call /all/takeoff crazyflie_interfaces/srv/Takeoff \"{height: $TAKEOFF_HEIGHT, duration: {sec: 2, nanosec: 0}}\"'" C-m
  fi

  # One key to stop everything, from any window:  Ctrl+b then K
  tmux bind-key K run-shell -b "cd '$PWD' && SESSION='$SESSION' CONF='$CONF' ./dev_tmux.sh stop"
  tmux set-option -t "$SESSION" status-right " Ctrl+b K = stop all | Ctrl+b d = detach "
  tmux set-option -t "$SESSION" status-right-length 60

  tmux select-window -t "$SESSION:ctrl"
  [ "$ATTACH" = "0" ] && { echo "session '$SESSION' started (not attached)."; return 0; }
  exec tmux attach -t "$SESSION"
}

# Physics-less particles in Gazebo: particle_sim integrates /cf_i/cmd_velocity_world, publishes
# /cf_i/state and moves coloured spheres in a zero-gravity world. No firmware, no crazyflie_server.
#   N_DRONES=5 SPH_MASS=auto SPH_H=0.3 SPH_SPACING=0.3 SPH_GAIN=0.05 ./dev_tmux.sh up-particles
# controller: PART_CTRL=plain (default, one node per drone) | swarm (one process for all)
# 3D: SPH_DIM=3 (sphere potential; POT_ZC, Z_MIN, Z_MAX, MAX_VZ, Z_STRETCH, PART_TAU_Z, PART_ZDRIFT, PART_ZJITTER)
# knobs: PART_LAYOUT (grid|ring, default grid) PART_TAU (0.15) PART_DELAY (0.1) PART_VIZ (gz|rviz|none) PART_VIZ_HZ (5) PART_WAIT (8) GZ_GUI (1)
up_particles() {
  resolve
  if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "A '$SESSION' tmux session is already running: $0 stop first."; exit 1
  fi
  local WORLD=src/particles_world.sdf
  local SPHERE_ARG=()                       # 3D: draw the potential sphere in the world
  [ "$SPH_DIM" = "3" ] && SPHERE_ARG=(--sphere "$POT_XC,$POT_YC,${POT_ZC:-1.0},$POT_R")
  python3 gen_config.py --n "$N_DRONES" --radius "$SPAWN_RADIUS" --layout "${PART_LAYOUT:-grid}" \
          --spacing "$SPH_SPACING" --config-dir config --world "$WORLD" --fixed-z "$FIXED_Z" \
          "${SPHERE_ARG[@]}" || exit 1
  local POS
  POS=$(tr -d ' ' < config/agents.txt | paste -sd/ -)
  fresh_container
  echo "$RUN_ID" > .last_run_id
  echo "run id: $RUN_ID  -> src/logs/runs/$RUN_ID   (analyse with: ./dev_tmux.sh analyze)"
  xhost +local:root >/dev/null 2>&1
  local PARAMS="-p total_robots:=$N_DRONES -p fixed_z:=$FIXED_Z -p initial_positions:=$POS"
  PARAMS="$PARAMS -p tau:=${PART_TAU:-0.15} -p delay:=${PART_DELAY:-0.1} -p world:=particles"
  PARAMS="$PARAMS -p viz:=${PART_VIZ:-gz} -p viz_hz:=${PART_VIZ_HZ:-5.0}"
  # vertical channel (3D): slower lag than x/y, climb limit, optional sinking drift, start-height jitter
  PARAMS="$PARAMS -p tau_z:=${PART_TAU_Z:-0.3} -p max_vz:=${MAX_VZ:-0.25} -p z_drift:=${PART_ZDRIFT:-0.0}"
  [ "$SPH_DIM" = "3" ] && PARAMS="$PARAMS -p z_jitter:=${PART_ZJITTER:-0.12}"

  tmux new-session -d -s "$SESSION" -n sim
  tmux send-keys -t "$SESSION:sim" \
    "$DC 'rm -rf /tmp/gz*; gz sim -s -r /root/dev_ws/src/particles_world.sdf'" C-m
  if [ "${GZ_GUI:-1}" = "1" ]; then
    tmux new-window -t "$SESSION" -n gui
    tmux send-keys -t "$SESSION:gui" "sleep 3; $DC 'gz sim -g'" C-m
  fi

  tmux new-window -t "$SESSION" -n ctrl
  tmux send-keys -t "$SESSION:ctrl" \
    "sleep ${PART_WAIT:-8}; $DC '$ROS_ENV && cd /root/dev_ws && colcon build --symlink-install && source src/dev_setup.bash && ros2 launch drone_controller $CTRL_LAUNCH $LAUNCH_ARGS'" C-m

  tmux new-window -t "$SESSION" -n part
  tmux send-keys -t "$SESSION:part" "sleep $((${PART_WAIT:-8} + 25)); $DC '$ROS_ENV && cd /root/dev_ws && source src/dev_setup.bash && ros2 run drone_controller particle_sim --ros-args $PARAMS'" C-m
  tmux new-window -t "$SESSION" -n rtf
  tmux send-keys -t "$SESSION:rtf" \
    "sleep 5; $DC '$ROS_ENV && python3 /root/dev_ws/src/tools/rtf_logger.py --out /root/dev_ws/src/logs/runs/$RUN_ID/rtf.csv'" C-m
  tmux new-window -t "$SESSION" -n sys
  tmux send-keys -t "$SESSION:sys" \
    "$DC 'python3 /root/dev_ws/src/tools/sys_monitor.py --out /root/dev_ws/src/logs/runs/$RUN_ID/sys.csv'" C-m

  tmux bind-key K run-shell -b "cd '$PWD' && SESSION='$SESSION' CONF='$CONF' ./dev_tmux.sh stop"
  tmux set-option -t "$SESSION" status-right " Ctrl+b K = stop all | Ctrl+b d = detach "
  tmux set-option -t "$SESSION" status-right-length 60
  tmux select-window -t "$SESSION:ctrl"
  [ "$ATTACH" = "0" ] && { echo "session '$SESSION' started (not attached)."; return 0; }
  exec tmux attach -t "$SESSION"
}

stop() {
  # ORDER MATTERS: when this runs from inside the tmux session (Ctrl+b K, or typed in a
  # window), killing the session kills this very script. So tmux kill-session is LAST.
  if tmux has-session -t "$SESSION" 2>/dev/null; then
    tmux send-keys -t "$SESSION:ctrl" C-c   # clean Ctrl+C so plain_sph flushes its CSVs
    sleep 3
  fi

  # Patterns use the [x] trick so pkill -f never matches the bash -lc running it
  # (otherwise it kills itself and the rest of the chain never runs).
  docker compose exec -T crazysim bash -lc "
    pkill -f '[p]lain_sph'
    pkill -f '[s]warm_sph'
    pkill -f '[p]article_sim'
    pkill -f '[r]tf_logger'
    pkill -f '[s]ys_monitor'
    pkill -f '[r]os2 launch'
    pkill -f '[s]itl_multiagent'
    pkill -f '[g]z sim'
    pkill -x cf2
    pkill -9 -x ruby
    chown -R $(id -u):$(id -g) /root/dev_ws/src/logs
    true" 2>/dev/null
  echo "container processes stopped, logs owned by $USER"
  [ -f .last_run_id ] && echo "last run: $(cat .last_run_id)   -> ./dev_tmux.sh analyze"
  docker compose restart -t 1 crazysim >/dev/null 2>&1 && echo "container restarted (clean slate for the next run)"

  tmux kill-session -t "$SESSION" 2>/dev/null
  echo "stopped."
}

check() {
  echo "== requested =="
  echo "N_DRONES=$N_DRONES"
  echo "== host config =="
  echo "agents.txt    : $(grep -c . config/agents.txt) lines"
  echo "crazyflies.yaml: $(grep -c '^  cf_[0-9]' config/crazyflies.yaml) robots"
  echo "== what the container sees =="
  docker compose exec -T crazysim sh -c "
    echo \"agents.txt        : \$(grep -c . $C_AGENTS) lines\"
    echo \"yaml (src mount)  : \$(grep -c '^  cf_[0-9]' $C_YAML_SRC) robots\"
    [ -e $C_YAML_INST ] && echo \"yaml (install)    : \$(grep -c '^  cf_[0-9]' $C_YAML_INST) robots\"; true"
  if mounts_ok; then echo "host and container files MATCH"
  else echo "MISMATCH -> stale bind mount. './dev_tmux.sh up' restarts the container and fixes it"
       echo "           (manual runs: docker compose restart crazysim)"; fi
  echo "== processes running in the container =="
  docker compose exec -T crazysim sh -c '
    echo "cf2 (firmware)  : $(pgrep -xc cf2)"
    echo "gz sim          : $(pgrep -fc "[g]z sim")"
    echo "crazyflie_server: $(pgrep -fc "[c]razyflie_server")"
    echo "rviz2           : $(pgrep -fc "[r]viz2")"
    echo "plain_sph       : $(pgrep -fc "[p]lain_sph")"'
  echo "== tmux =="
  tmux ls 2>&1
}

analyze() {
  local id="${1:-$( [ -f .last_run_id ] && cat .last_run_id )}"
  local dir="src/logs/runs/$id"
  [ -n "$id" ] && [ -d "$dir" ] || { echo "run folder '$dir' not found (ls src/logs/runs)"; exit 1; }
  python3 src/analyze_run.py "$dir"
}

case "${1:-up}" in
  up)     up ;;
  up-swarm) CTRL_LAUNCH=swarm_sph.launch.py; up ;;
  up-particles) CTRL_LAUNCH="${PART_CTRL:-plain}_sph.launch.py"; up_particles ;;
  analyze) shift; analyze "$@" ;;
  attach) exec tmux attach -t "$SESSION" ;;
  stop)   stop ;;
  show)   show ;;
  check)  check ;;
  *)      echo "usage: $0 {up|up-swarm|up-particles|attach|stop|show|check|analyze [run_id]}"; exit 1 ;;
esac