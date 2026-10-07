#!/usr/bin/env bash
# CPU / memory / RTF benchmark: `./dev_tmux.sh up` (one controller process per drone) against
# `./dev_tmux.sh up-swarm` (one process for all), for several drone counts. Each run is short and
# calm (hover, zero command) so only the machine load differs.
#
#   tests/bench_cpu.sh                         modes "up up-swarm", N = 2 3 5, 75 s of flight each
#   N_LIST="1 2 3" MODES="up-swarm" tests/bench_cpu.sh
#   RVIZ=True tests/bench_cpu.sh               include the cost of RViz (default: off)
#   FLIGHT_SECONDS=120 tests/bench_cpu.sh
#
# Each run writes src/logs/runs/<id>/ with sys.csv (CPU per process group, memory, clock, temperature),
# rtf.csv and the controller logs. At the end: one table (src/logs/runs/<batch>_table.md) with RTF,
# busy cores per group, memory and clock for every run (made by src/tools/sys_report.py).
set -u
cd "$(dirname "$0")/.." || exit 1
N_LIST="${N_LIST:-2 3 5}"
MODES="${MODES:-up up-swarm}"
FLIGHT="${FLIGHT_SECONDS:-75}"
export RVIZ="${RVIZ:-False}" START_GATE=flight TEST_PROFILE=zero TEST_DELAY=3 POT_K=0
eval "$(bash -c "source swarm.conf; echo SIM_WAIT=\$SIM_WAIT SWARM_WAIT=\$SWARM_WAIT TAKEOFF_WAIT=\$TAKEOFF_WAIT")"
BATCH="$(date +%Y%m%d_%H%M%S)_bench"
mkdir -p src/logs/runs 2>/dev/null
IDS=()
for mode in $MODES; do
  for n in $N_LIST; do
    tag=swarm; [ "$mode" = "up" ] && tag=plain
    label="bench_${tag}_N${n}"
    wait_s=$((SIM_WAIT + SWARM_WAIT + TAKEOFF_WAIT + FLIGHT + 10))
    echo "=== $label: mode=$mode N=$n rviz=$RVIZ, ${wait_s}s"
    export N_DRONES=$n RUN_LABEL="$label" RUN_ID="$(date +%Y%m%d_%H%M%S)_${label}" DETACH=1
    ./dev_tmux.sh "$mode" || { echo "start failed, skipping"; continue; }
    sleep "$wait_s"
    ./dev_tmux.sh stop
    sleep 3
    python3 src/tools/sys_report.py "src/logs/runs/$RUN_ID"
    IDS+=("$RUN_ID")
    sleep 5
  done
done
if [ "${#IDS[@]}" -gt 0 ]; then
  args=(); for i in "${IDS[@]}"; do args+=("src/logs/runs/$i"); done
  python3 src/tools/sys_report.py --md "src/logs/runs/${BATCH}_table.md" "${args[@]}"
  echo "table: src/logs/runs/${BATCH}_table.md"
fi