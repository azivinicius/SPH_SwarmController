#!/usr/bin/env bash
# dev_tmux.sh -- run the whole SPH_Project stack from ONE host terminal, driven by swarm.conf.
# Run on the HOST from ~/SPH_Project.
#
#   ./dev_tmux.sh up      generate configs for N_DRONES, start container + tmux session, attach
#   ./dev_tmux.sh attach  re-attach to the running session
#   ./dev_tmux.sh stop    Ctrl+C the controller (flushes CSVs), stop everything, fix log ownership.
#                         Also bound to  Ctrl+b K  inside the session (works from any window).
#   ./dev_tmux.sh show    print the resolved config and the launch command, then exit
#   ./dev_tmux.sh check   compare host config files with what the container actually sees,
#                         and count running sim processes (use this when a run looks wrong)
#
# Per-run overrides:  N_DRONES=5 POT_R=0.8 ./dev_tmux.sh up
# Other config file:  CONF=configs/five.conf ./dev_tmux.sh up

SESSION="${SESSION:-sph}"
CONF="${CONF:-swarm.conf}"
ATTACH="${ATTACH:-1}"        # 0 = start the session but do not attach (scripts/tests)

cd "$(dirname "$0")" || exit 1
[ -f "$CONF" ] || { echo "config file '$CONF' not found"; exit 1; }
# shellcheck disable=SC1090
source "$CONF"

DC="docker compose exec crazysim bash -lc"
# 'bash -lc' is non-interactive, so it does NOT read /root/.bashrc, which is where the
# Dockerfile sources ROS. Source it explicitly in every command that needs ros2.
ROS_ENV="source /opt/ros/humble/setup.bash && source /CrazySim/crazyswarm2_ws/install/setup.bash"

LAUNCH_ARGS="total_robots:=$N_DRONES"

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

show() {
  echo "config file : $CONF"
  echo "drones      : $N_DRONES (spawn radius $SPAWN_RADIUS m)"
  echo "launch args : $LAUNCH_ARGS"
}

up() {
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

  # dev_setup.bash, NOT install/setup.bash (README known issue #1)
  tmux new-window -t "$SESSION" -n ctrl
  tmux send-keys -t "$SESSION:ctrl" \
    "sleep $((SIM_WAIT + SWARM_WAIT)); $DC '$ROS_ENV && cd /root/dev_ws && colcon build --symlink-install && source src/dev_setup.bash && ros2 launch drone_controller plain_sph.launch.py $LAUNCH_ARGS'" C-m

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
    pkill -f '[r]os2 launch'
    pkill -f '[s]itl_multiagent'
    pkill -f '[g]z sim'
    pkill -x cf2
    pkill -9 -x ruby
    chown -R $(id -u):$(id -g) /root/dev_ws/src/logs
    true" 2>/dev/null
  echo "container processes stopped, logs owned by $USER"
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

case "${1:-up}" in
  up)     up ;;
  attach) exec tmux attach -t "$SESSION" ;;
  stop)   stop ;;
  show)   show ;;
  check)  check ;;
  *)      echo "usage: $0 {up|attach|stop|show|check}"; exit 1 ;;
esac