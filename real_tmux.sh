#!/usr/bin/env bash
# real_tmux.sh -- REAL Crazyflies: Crazyswarm2 (cflib, Crazyradio) + your drone_controller. No SITL, no Gazebo.
# Run on the HOST from ~/SPH_Project.  Own tmux session ('real'): never touches the simulation session.
# Enabled robots in the real yaml MUST be cf_0 .. cf_{N-1} (the controller builds the names from N).
#
#   ./real_tmux.sh up                               start Crazyswarm2 with config/crazyflies_real.yaml, attach
#   HL_HEIGHT=0.5 ./real_tmux.sh takeoff            all enabled drones take off
#   TEST=zero|step|sine|... ./real_tmux.sh ctrl     test profile through YOUR controller (SPH off)
#   SQ_SIDE=1.0 SQ_STEP=6 ./real_tmux.sh square     box profile: closed-loop square, N = 1..4 (drone i -> vertex i)
#   POT_R=0.51 POT_K=10 ./real_tmux.sh sph          SPH swarm; every SPH/potential knob as in dev_tmux.sh
#                                                   (the launch args come from ./dev_tmux.sh args)
#   ./real_tmux.sh land                             land (through the controller if it runs, firmware as backup)
#   ./real_tmux.sh stop                             LAND, then close Crazyswarm2 and the session
# knobs: REAL_YAML, HL_HEIGHT (0.5), MAX_VELOCITY (0.2), FENCE_* (+-1.5), SQ_XC SQ_YC (0,0), SQ_STEP (6 s),
#        SQ_FIRST (8 s), SQ_SIDE (1.0), TEST_AMP (0.2), TEST_DURATION (40 s)

SESSION="${SESSION:-real}"
REAL_YAML="${REAL_YAML:-config/crazyflies_real.yaml}"
ATTACH="${ATTACH:-1}"
cd "$(dirname "$0")" || exit 1

DC="docker compose exec crazysim bash -lc"
ROS_ENV="source /opt/ros/humble/setup.bash && source /CrazySim/crazyswarm2_ws/install/setup.bash"

real_names() {   # enabled robots of the real yaml, in file order
  awk '/^robots:/ {r = 1; next}  r && /^[^ #]/ {r = 0}
       r && /^  [A-Za-z0-9_]+:/ {n = $1; sub(":", "", n)}
       r && /^    enabled: *true/ {print n}' "$REAL_YAML"
}

hl_call() { docker compose exec -T crazysim bash -lc "$ROS_ENV && $1"; }

check_names() {  # the controller expects cf_0 .. cf_{n-1}
  local i=0 nm
  for nm in $(real_names); do
    [ "$nm" = "cf_$i" ] || { echo "enabled robots must be cf_0..cf_{N-1} in order (found $nm at position $i) in $REAL_YAML"; exit 1; }
    i=$((i + 1))
  done
  [ "$i" -ge 1 ] || { echo "no enabled robots in $REAL_YAML"; exit 1; }
}

start_ctrl() {   # $1 = launch args: (re)open the 'ctrl' window and run plain_sph
  tmux has-session -t "$SESSION" 2>/dev/null || { echo "session '$SESSION' not running: $0 up first"; exit 1; }
  tmux list-windows -t "$SESSION" -F '#W' | grep -qx ctrl && { echo "a 'ctrl' window is already running: land / stop first"; exit 1; }
  tmux new-window -t "$SESSION" -n ctrl
  tmux send-keys -t "$SESSION:ctrl" "$DC '$ROS_ENV && cd /root/dev_ws && colcon build --symlink-install && source src/dev_setup.bash && ros2 launch drone_controller plain_sph.launch.py $1'" C-m
}

up() {
  [ -f "$REAL_YAML" ] || { echo "$REAL_YAML not found: copy config/crazyflies_real.example.yaml and edit it"; exit 1; }
  tmux has-session -t "$SESSION" 2>/dev/null && { echo "session '$SESSION' already running: $0 stop first"; exit 1; }
  tmux has-session -t sph 2>/dev/null && { echo "the SIMULATION session 'sph' is running: ./dev_tmux.sh stop first"; exit 1; }
  cp "$REAL_YAML" config/crazyflies.yaml            # in place: the container's bind mount sees it
  docker compose up -d crazysim || exit 1
  docker compose restart -t 1 crazysim >/dev/null || exit 1   # clean process space (drones are on the ground now)
  echo "drones: $(real_names | paste -sd' ' -)"
  tmux new-session -d -s "$SESSION" -n swarm
  tmux send-keys -t "$SESSION:swarm" "$DC '$ROS_ENV && ros2 launch crazyflie launch.py backend:=cflib rviz:=False'" C-m
  tmux new-window -t "$SESSION" -n shell
  tmux bind-key l run-shell -b "cd '$PWD' && SESSION='$SESSION' ./real_tmux.sh land"
  tmux set-option -t "$SESSION" status-right " REAL DRONES | Ctrl+b l = land all "
  tmux set-option -t "$SESSION" status-right-length 60
  tmux select-window -t "$SESSION:shell"
  [ "$ATTACH" = "0" ] && { echo "session '$SESSION' started (not attached)."; return 0; }
  exec tmux attach -t "$SESSION"
}
ctrl() {
  check_names
  local n="${N:-$(real_names | wc -l)}" prof="${TEST:-zero}"
  [ "$prof" = "box" ] && [ "$n" -gt 4 ] && { echo "box: at most 4 drones (one per vertex), $n enabled"; exit 1; }
  local A="total_robots:=$n platform:=real sim_clock:=wall start_gate:=flight stale_policy:=hold"
  A="$A fixed_z:=${HL_HEIGHT:-0.5} max_velocity:=${MAX_VELOCITY:-0.2} max_vz:=${MAX_VZ:-0.2} state_timeout_s:=1.0"
  A="$A fence_xmin:=${FENCE_XMIN:--1.5} fence_xmax:=${FENCE_XMAX:-1.5} fence_ymin:=${FENCE_YMIN:--1.5} fence_ymax:=${FENCE_YMAX:-1.5}"
  A="$A sph_gain:=0.0 f_k:=0.0 test_profile:=$prof test_amp:=${TEST_AMP:-0.2} test_period:=${SQ_STEP:-6.0}"
  A="$A test_delay:=${SQ_FIRST:-8.0} test_duration:=${TEST_DURATION:-40.0} test_side:=${SQ_SIDE:-1.0}"
  A="$A xc:=${SQ_XC:-0.0} yc:=${SQ_YC:-0.0} run_label:=${RUN_LABEL:-real_${prof}_${n}}"
  start_ctrl "$A"
  echo "controller started (window 'ctrl'): TEST=$prof, $n drone(s)"
}

sph() {   # the SPH swarm on the real drones: same launch args as the simulation (dev_tmux.sh resolve)
  check_names
  local n; n=$(real_names | wc -l)
  local A
  A=$(N_DRONES=$n PLATFORM=real SIM_CLOCK=wall START_GATE=flight STALE_POLICY=hold STATE_TIMEOUT_S=1.0 \
      FIXED_Z=${HL_HEIGHT:-0.5} MAX_VELOCITY=${MAX_VELOCITY:-0.2} MAX_VZ=${MAX_VZ:-0.2} \
      POT_XC=${POT_XC:-0.0} POT_YC=${POT_YC:-0.0} TEST_PROFILE=none \
      FENCE_XMIN=${FENCE_XMIN:--1.5} FENCE_XMAX=${FENCE_XMAX:-1.5} FENCE_YMIN=${FENCE_YMIN:--1.5} FENCE_YMAX=${FENCE_YMAX:-1.5} \
      RUN_LABEL=${RUN_LABEL:-real_sph_${n}} ./dev_tmux.sh args) || { echo "./dev_tmux.sh args failed"; exit 1; }
  echo "launch args: $A"
  start_ctrl "$A"
  echo "SPH controller started (window 'ctrl'): $n drone(s)"
}
takeoff() {
  local z; z=$(awk -v v="${HL_HEIGHT:-0.5}" 'BEGIN {printf "%.3f", v}')
  hl_call "ros2 service call /all/takeoff crazyflie_interfaces/srv/Takeoff \"{height: $z, duration: {sec: 3, nanosec: 0}}\""
}

land() {   # with the controller running: through YOUR Commander.land; the firmware land is the backup
  if tmux list-windows -t "$SESSION" -F '#W' 2>/dev/null | grep -qx ctrl; then
    hl_call "ros2 topic pub -w 1 -t 5 -r 5 /sph/land std_msgs/msg/Empty '{}'" >/dev/null \
      && echo "land requested through the controller (Commander.land)"
    sleep 1
  fi
  hl_call "ros2 service call /all/land crazyflie_interfaces/srv/Land \"{height: 0.0, duration: {sec: 3, nanosec: 0}}\""
}

square() { TEST=box ctrl; }

stop() {
  land && sleep 4                                    
  docker compose exec -T crazysim bash -lc "pkill -INT -f '[p]lain_sph'; sleep 2; pkill -f '[c]razyflie_server'; pkill -f '[r]os2 launch'; true" 2>/dev/null
  tmux kill-session -t "$SESSION" 2>/dev/null
  echo "stopped (container NOT restarted)."
}

case "${1:-up}" in
  up)      up ;;    
  takeoff) takeoff ;;
  land)    land ;;
  square)  square ;;
  stop)    stop ;;
  ctrl)    ctrl ;;
  sph)     sph ;;
  attach)  exec tmux attach -t "$SESSION" ;;
  *) echo "usage: $0 up | takeoff | ctrl | square | sph | land | stop | attach"; exit 1 ;;
esac