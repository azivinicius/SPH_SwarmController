#!/usr/bin/env bash
# dev_tmux.sh -- run the whole SPH_Project stack from ONE host terminal, driven by swarm.conf.
# Run on the HOST from ~/SPH_Project.
#
#   ./dev_tmux.sh up      generate configs for N_DRONES, start container + tmux session, attach
#   ./dev_tmux.sh attach  re-attach to the running session
#   ./dev_tmux.sh stop    Ctrl+C the controller (flushes CSVs), stop everything, fix log ownership.
#                         Also bound to  Ctrl+b K  inside the session (works from any window).
#   ./dev_tmux.sh show    print the resolved config and the launch command, then exit
#
# Per-run overrides:  N_DRONES=5 POT_R=0.8 ./dev_tmux.sh up
# Other config file:  CONF=configs/five.conf ./dev_tmux.sh up

SESSION="${SESSION:-sph}"
CONF="${CONF:-swarm.conf}"

cd "$(dirname "$0")" || exit 1
[ -f "$CONF" ] || { echo "config file '$CONF' not found"; exit 1; }
# shellcheck disable=SC1090
source "$CONF"

DC="docker compose exec crazysim bash -lc"
# 'bash -lc' is non-interactive, so it does NOT read /root/.bashrc, which is where the
# Dockerfile sources ROS. Source it explicitly in every command that needs ros2.
ROS_ENV="source /opt/ros/humble/setup.bash && source /CrazySim/crazyswarm2_ws/install/setup.bash"

LAUNCH_ARGS="total_robots:=$N_DRONES fixed_z:=$FIXED_Z \
particle_mass:=$SPH_MASS smoothing_h:=$SPH_H rho_0:=$SPH_RHO0 gamma:=$SPH_GAMMA \
alpha:=$SPH_ALPHA beta:=$SPH_BETA zetta_extra:=$SPH_ZETTA \
xc:=$POT_XC yc:=$POT_YC R:=$POT_R f_k:=$POT_K f_beta:=$POT_BETA"

show() {
  echo "config file : $CONF"
  echo "drones      : $N_DRONES (spawn radius $SPAWN_RADIUS m)"
  echo "launch args : $LAUNCH_ARGS"
}

up() {
  python3 gen_config.py --n "$N_DRONES" --radius "$SPAWN_RADIUS" --config-dir config || exit 1
  docker compose up -d crazysim || exit 1
  xhost +local:root >/dev/null 2>&1

  if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Session '$SESSION' already exists -- attaching. (Use '$0 stop' first for a clean start;"
    echo "the freshly generated configs only take effect on a new start.)"
    exec tmux attach -t "$SESSION"
  fi

  tmux new-session -d -s "$SESSION" -n sim
  tmux send-keys -t "$SESSION:sim" \
    "$DC 'cd /CrazySim/crazyflie-firmware && bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_multiagent_headless.sh -f agents.txt'" C-m

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

  tmux kill-session -t "$SESSION" 2>/dev/null
  echo "stopped."
}

case "${1:-up}" in
  up)     up ;;
  attach) exec tmux attach -t "$SESSION" ;;
  stop)   stop ;;
  show)   show ;;
  *)      echo "usage: $0 {up|attach|stop|show}"; exit 1 ;;
esac