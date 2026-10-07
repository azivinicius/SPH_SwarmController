#!/usr/bin/env bash
# tests/run_test.sh <ID> [VAR=value ...]
#
# Runs ONE experiment of TEST_GUIDE.md unattended:
#   start simulator + controller (detached) -> wait -> stop -> analyse -> print summary
# Run it on the HOST from ~/SPH_Project (same place as dev_tmux.sh).
#
#   tests/run_test.sh T1                    baseline
#   tests/run_test.sh T2                    hover (zero command)
#   tests/run_test.sh T4                    gain sweep (4 runs)
#   tests/run_test.sh T1 N_DRONES=5         any swarm.conf variable can be overridden
#   FLIGHT_SECONDS=90 tests/run_test.sh T1  longer flight window
#   DT_LIST="0.05 0.03 0.022" tests/run_test.sh T7   choose the DT_MODEL values of T7
#
# IDs: T1 T2 T3a T3b T4 T5 T7 T8 T9 T10 T11 T13   (single runs: T1 T2 T3a T3b; sweeps run several)
# Defaults: N_DRONES=5, POT_K=0 (the setup of the logs analysed so far).
# Each run ends up in src/logs/runs/<timestamp>_<label>/ ; summary in .../analysis/summary.txt
set -u
cd "$(dirname "$0")/.." || exit 1
ID="${1:?usage: tests/run_test.sh <T1|T2|T3a|T3b|T4|T5|T7|T8|T9|T10|T11|T13> [VAR=value ...]}"
shift
for kv in "$@"; do export "$kv"; done

CONF="${CONF:-swarm.conf}"
# read ONLY the startup waits from the config (sourcing it here would set every variable and make the
# "default only if the caller did not set it" helper below useless)
eval "$(bash -c "source '$CONF'; echo SIM_WAIT=\$SIM_WAIT SWARM_WAIT=\$SWARM_WAIT TAKEOFF_WAIT=\$TAKEOFF_WAIT")"
FLIGHT="${FLIGHT_SECONDS:-60}"        # seconds of flight to record after takeoff
IDS_FILE="${IDS_FILE:-/dev/null}"     # run_all.sh collects run ids here

# default only if the caller did not set it
d() { eval ": \"\${$1:=$2}\""; export "${1?}"; }

# Defaults of every test = the configuration of the logs analysed so far: 5 drones, potential force OFF.
# (override on the command line:  tests/run_test.sh T1 N_DRONES=3 POT_K=2)
d N_DRONES 5
d POT_K 0

# one run: start, wait, stop, analyse.  Uses the exported variables as they are.
leaf() {
  # sim clock: the flight takes 1/RTF times longer in wall seconds; SLOWDOWN = assumed 1/RTF (default 4)
  local flight_wall=$FLIGHT
  [ "${SIM_CLOCK:-wall}" = gz ] && flight_wall=$((FLIGHT * ${SLOWDOWN:-4}))
  local wait_s=$((SIM_WAIT + SWARM_WAIT + TAKEOFF_WAIT + flight_wall + 10))
  echo "=== $RUN_LABEL: ${wait_s}s (N=${N_DRONES:-3} gate=${START_GATE:-none} profile=${TEST_PROFILE:-none}" \
       "clock=${SIM_CLOCK:-wall} gain=${SPH_GAIN:-1.0} policy=${STALE_POLICY:-predict} dt_model=${DT_MODEL:-0.0} rviz=${RVIZ:-conf})"
  RUN_ID="$(date +%Y%m%d_%H%M%S)_${RUN_LABEL}" DETACH=1 ./dev_tmux.sh up-swarm || return 1
  sleep "$wait_s"
  ./dev_tmux.sh stop
  sleep 3
  ./dev_tmux.sh analyze
  [ -f .last_run_id ] && cat .last_run_id >> "$IDS_FILE"
  sleep 5
}

# run the same leaf with one variable swept over values:  sweep VAR label_prefix v1 v2 ...
sweep() {
  local var="$1" prefix="$2"; shift 2
  local v
  for v in "$@"; do
    ( export "$var=$v" RUN_LABEL="${prefix}_${v}"; leaf )
  done
}

case "$ID" in
  T1)   d RUN_LABEL T1_baseline;                                   leaf ;;
  T2)   d RUN_LABEL T2_hover;  d START_GATE flight; d TEST_PROFILE zero; d TEST_DELAY 3
        FLIGHT=$(( ${FLIGHT_SECONDS:-45} ));                       leaf ;;
  T3a)  d RUN_LABEL T3a_step;  d START_GATE flight; d TEST_PROFILE step; d TEST_AMP 0.2; d TEST_PERIOD 5
        FLIGHT=$(( 3 + 5 + 10 + 10 ));                             leaf ;;
  T3b)  d RUN_LABEL T3b_chirp; d START_GATE flight; d TEST_PROFILE chirp; d TEST_AMP 0.15; d TEST_DURATION 60
        FLIGHT=$(( 3 + 60 + 10 ));                                 leaf ;;
  T4)   sweep SPH_GAIN T4_gain 0 0.25 0.5 1.0 ;;
  T5)   for n in 1 2 3; do for rv in True False; do
          ( export N_DRONES=$n RVIZ=$rv RUN_LABEL="T5_N${n}_rviz${rv}"; leaf ); done; done ;;
  T7)   # shellcheck disable=SC2086
        sweep DT_MODEL T7_dt ${DT_LIST:-0.05 0.03 0.02 0.0125} ;;
  T8)   sweep STALE_POLICY T8_policy predict hold skip ;;
  T9)   sweep START_GATE T9_gate none all_known flight ;;
  T10)  sweep N_DRONES T10_N 3 5 7 ;;
  T11)  sweep POT_K T11_fk 0 2 ;;
  T13)  d RUN_LABEL T13_simclock; d SIM_CLOCK gz; d START_GATE flight
        FLIGHT=$(( ${FLIGHT_SECONDS:-60} ));                       leaf ;;
  *)    echo "unknown test id '$ID'"; exit 1 ;;
esac
