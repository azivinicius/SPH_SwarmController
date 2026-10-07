# SPH swarm — test guide

How to find out **what is really happening** when the drones drift or shake: the instruments
(logs + analyzer), the hypotheses, and a battery of experiments that each isolate one suspect.

Everything below runs on your host from `~/SPH_Project`. Commands are copy-paste.

---

## 0. The 60-second version

```bash
# 1. one-time: check the instruments (no simulator needed, ~10 s)
python3 tests/test_core_equivalence.py && python3 tests/test_analyzer.py && python3 tests/test_sim_clock.py

# 2. the three tests that split the problem into "simulator timing / drone / SPH" (~8 min)
tests/run_all.sh quick           # = T1 baseline, T2 hover, T3a step response

# 3. open the one-page verdict of each run and send it to me
cat src/logs/runs/*T1_baseline/analysis/summary.txt
cat src/logs/runs/*T2_hover/analysis/summary.txt
cat src/logs/runs/*T3a_step/analysis/summary.txt
cat src/logs/runs/compare_*_quick/compare.md
```

**Low RTF with many drones?** Skip straight to the fix: `tests/run_test.sh T13` (controller on simulated time, §T13), or `SIM_CLOCK=gz` in `swarm.conf`.
**Real drones?** See `REAL_DRONES.md`.

`summary.txt` is written to be pasted into a chat: verdicts first, then a ranked diagnosis.
The figures next to it (`analysis/fig_*.png`) are what to look at yourself.

---

## 1. What we know, what we suspect

### 1.1 Facts taken from your files

| Fact | Source |
|---|---|
| Gazebo runs far below real time: RTF **mean 0.38**, median 0.39, 5–95 % band 0.17–0.56, min 0.05, max 0.73 (427 samples, 5 drones) | `src/logs/rtf_log.txt` |
| Controller timer = 20 Hz of **wall** time (`CONTROL_PERIOD = 0.05`), state topic = 20 Hz | `plain_sph.py`, `config/crazyflies.yaml` |
| Each tick the controller integrates `v_cmd = v_meas + dv/dt · 0.05` and sends it as a world-frame velocity command (`vz = 0`) | `particle.py`, `commander.py` |
| 5-drone ring, radius 1.51 m, f_k = 0, zeta = 10, gamma = 2, mass 7000. A slow growing oscillation at ≈1.1 Hz (wall), amplitude growing ≈ 9 %/s; mean ring radius 1.51 → 1.63 m in 37 s; `cf_3` moved 0.50 m | `analyze_run.py` run on your old CSVs (limited analysis: only the command was logged, not the measured velocity) |

### 1.2 Hypotheses (what each test is designed to separate)

| # | Hypothesis | Mechanism | Settled by |
|---|---|---|---|
| **H1** | **Time-base mismatch** | The simulator runs at RTF ≈ 0.4 but the controller ticks in wall time. Each tick covers only `RTF × 50 ms = 20 ms` of simulated time while the SPH model integrates `dt = 50 ms`. Per **simulated** second the controller applies `1/RTF ≈ 2.5×` the acceleration the SPH model intends, and delays measured in ticks become `1/RTF` times longer in simulated time. A loop that is stable at RTF 1 can oscillate at RTF 0.4. | T1 (measures RTF), T5, **T7** |
| **H2** | **Stale state** | If the 20 Hz state topic is paced by simulated time, it arrives at `20 × RTF ≈ 8 Hz` of wall time, so ≈ 60 % of 20 Hz ticks have no new message and integrate the controller's own prediction again. (Your old logs suggested ≈ 25 %, so either RTF was higher in that run or the topic is not sim-paced. T1 measures it directly.) | T1, **T8** |
| **H3** | **The drone does not follow velocity commands** | Firmware / estimator / SITL adds lag, gain error or a velocity bias. The controller closed around it has less phase margin, and the drone drifts by itself. | **T2**, **T3a/b** |
| **H4** | **The SPH loop itself is too stiff for the drone lag** | Pressure gradient (mass, gamma) acts like a spring on a drone that tracks velocity with lag → oscillation above a critical gain. | **T4** (gain sweep) |
| **H5** | **Startup transient** | The controller runs on the ground before takeoff and with only some drones known (rho dips to 550, P to −7700 in your first seconds) | T9 |
| **H6** | **Scaling with N** | More drones → lower RTF (H1) and more neighbours | T5, T10 |
| **H7** | **External potential** interacts with the loop | f_k > 0 | T11 |

H1–H3 are the most likely and cheapest to test: do T1, T2, T3a first.

---

## 2. Setup (once)

```bash
cd ~/SPH_Project
unzip -o SPH_diag_update.zip -d .           # the files I changed/added (see Appendix C)
git status --short                           # review: nothing of yours should be silently replaced
python3 tests/test_core_equivalence.py       # expect: ALL OK
python3 tests/test_analyzer.py               # expect: ALL OK
```

`python3` on the host needs `numpy` and `matplotlib` (you already use them for `plot_plain_sph.py`).

**Instrument check without Gazebo (T0)** — the real control loop against a toy drone:

```bash
python3 src/offline_twin.py --n 5 --seconds 60 --rtf 0.4 --label twin_rtf04
python3 src/analyze_run.py src/twin_logs/runs/*twin_rtf04
```

You should see `C1`/`C3` FAIL (RTF 0.40, ≈ 60 % stale ticks) because that is what the twin was told to do —
this proves the analyzer sees H1/H2 when they are present. Try `--rtf 1.0` and watch both disappear.
Other what-ifs: `--tau 0.4 --delay 0.45` (sluggish drone), `--bias 0.03 --bias-drones 2,3` (two drones drift
outward), `--test_profile step --start_gate flight` (the T3a protocol). The twin is a model, not Gazebo:
it validates the *instruments* and shows the *fingerprint* of each cause; it does not replace T1–T11.

**Smoke test on the real simulator (2 drones, ~1.5 min)** — do this before the battery:

```bash
tests/run_test.sh T1 N_DRONES=2 FLIGHT_SECONDS=20
```

Then check: `src/logs/runs/<id>/` contains `tick_cf_0.csv`, `rx_cf_0.csv`, `events.log`, `meta.json`,
`rtf.csv`, `analysis/summary.txt`. If `rtf.csv` is missing see §8.

Run by hand (instead of unattended) any time:

```bash
RUN_LABEL=my_experiment SPH_GAIN=0.5 ./dev_tmux.sh up-swarm     # starts everything, attaches to tmux
./dev_tmux.sh stop                                              # (or Ctrl+b K)
./dev_tmux.sh analyze                                           # analyses the last run
```

---

## 3. The instruments

### 3.1 One run = one folder: `src/logs/runs/<timestamp>_<label>/`

| File | What it is | Rows |
|---|---|---|
| `meta.json` | every parameter of the run (SPH, potential, switches, test profile, git revision) — so a run is never ambiguous | 1 |
| `tick_<drone>.csv` | **one row per control tick per drone**: what the controller saw, computed and sent | 20 / s |
| `rx_<drone>.csv` | **one row per state message received** (timing and values straight from the drone) | ≈ 20·RTF / s |
| `events.log` | human-readable timeline: start, first message of each drone, start-gate, warnings, 5-second heartbeats | few |
| `rtf.csv` | Gazebo real-time factor with wall-clock timestamps (written by `src/tools/rtf_logger.py`) | 4 / s |
| `analysis/` | `summary.txt` (paste this), `report.md`, `report.json`, `fig_*.png` — made by `analyze_run.py` | |

The old `logs/plain_sph_<drone>.csv` (9 columns) is still written, so `plot_plain_sph.py` keeps working.

### 3.2 `events.log` — read it top to bottom

```
    0.000 INFO  swarm   START   swarm: 5 drones, ..., sph_gain=1.0 zeta=10.0 f_k=0.0 gamma=2.0 m=7000.0 policy=predict gate=none ...
    0.008 INFO  cf_0    FIRST_RX first state of cf_0: x=1.510 y=0.000 z=0.000 (1/5 drones known)
   12.181 INFO  -       GATE_OPEN start gate "flight" open: controller now acts
   15.002 INFO  cf_3    HB      rho= 1000.0 P= -4.5 |v|rms=0.014 r_pot=1.52 stale=59.9% rx= 8.0Hz clip= 0.0% dt_max=60ms age_max=124ms
   18.716 WARN  cf_0    LATE_TICK tick came 113 ms after the previous one (nominal 50 ms)
   22.310 WARN  cf_2    RHO_LOW density 790 = 0.79 x rho_0 (P=-6100, n_nb=4, nearest 1.62 m)
```

| Code | Level | Meaning |
|---|---|---|
| `START`, `STOP` | INFO | parameters; at the end: ticks, % stale, % clipped, late ticks |
| `FIRST_RX` | INFO | first state of a drone arrived (how many drones are known) |
| `GATE_WAIT` / `GATE_OPEN` | INFO | start gate waiting (who is missing / still on the ground) / opened |
| `TEST_START` / `TEST_END` | INFO | a test profile began / finished |
| `HB` | INFO | every 5 s per drone: mean rho, P, speed rms, distance to potential centre, % stale ticks, **state rate in Hz**, % clipped, worst tick gap and state age |
| `LATE_TICK` | WARN | the control timer fired > 1.5 periods after the previous tick (CPU stall) |
| `SLOW_STEP` | WARN | one control step cost > 50 % of the period |
| `STALE_RUN` | WARN | ≥ 3 consecutive ticks without a state message from a drone |
| `PRED_ERR` | WARN | predicted position is > 5 cm from the last measured one |
| `CLIP_START` / `CLIP_END` | WARN/INFO | speed command saturated (0.5 m/s) for ≥ 5 ticks |
| `RHO_LOW` / `RHO_HIGH` / `RHO_OK` | WARN/INFO | density left / re-entered ±20 % of rho_0 (with P, neighbours, nearest distance) |

WARNs are rate-limited and say how many were suppressed. They are echoed to the `ctrl` tmux window too.

### 3.3 `tick_<drone>.csv` columns (one row per control tick)

| Group | Columns | Meaning |
|---|---|---|
| when | `t` `epoch` `tick` `drone` | seconds since node start; unix time; tick counter |
| timing | `wall_dt_ms` `step_ms` | time since previous tick (nominal 50); cost of the control step |
| mode | `gate` `src` | gate open?; command source: `sph`, `sph_pred` (stale tick, prediction), `hold`, `skip`, `gate` (not allowed to act yet), `test:<profile>` |
| freshness | `rx_since` `state_age_ms` `stale_run` `n_known` `n_nb_fresh` `max_nb_age_ms` | messages of this drone since last tick (**0 = stale tick**); age of its last message; consecutive stale ticks; neighbours known / fresh |
| measured | `meas_x/y/z` `meas_vx/vy/vz` | last message from the drone's own estimator |
| used | `used_x/y` `used_vx/vy` `pred_err` | state SPH really used (= measured on fresh ticks, **prediction** on stale ones); the gap |
| SPH | `rho` `P` `c` `n_nb` `nn_dist` `mean_nb_dist` | density, pressure, sound speed, neighbours, nearest / mean neighbour distance |
| accel budget | `ap_*` `av_*` `ad_*` `ae_*` `a_*` | pressure, artificial viscosity, damping (−ζv), external force, total dv/dt (m/s²) |
| command | `cmd_raw_*` `cmd_*` `clipped` `prev_cmd_*` | velocity before / after the 0.5 m/s clip; clip flag; previous command |
| geometry | `cen_x/y` `r_cen` `r_pot` | centroid of known drones; distance to centroid; distance to the potential centre |

`rx_<drone>.csv`: `t epoch drone rx_dt_ms x y z vx vy vz hdr_lat_ms` (`hdr_lat_ms` blank if the message has no stamp).

### 3.4 The analyzer

```bash
./dev_tmux.sh analyze                       # last run
python3 src/analyze_run.py src/logs/runs/latest
python3 src/analyze_run.py A B C --compare  # side-by-side table + overlay plot
python3 src/analyze_run.py "src/logs/plain_sph_cf_*.csv"   # old format, limited
```

**Analysed window:** from 2 s after *flight start* (all drones above 80 % of target altitude for 1 s) to the end;
for a test profile, from the first test sample. The window is printed at the top of every report, so
numbers are comparable between runs. Change the settling time with `--skip`.

**Health checks** (thresholds in `TH` at the top of `analyze_run.py`):

| Id | Check | Looks at | PASS means | WARN/FAIL → go to |
|---|---|---|---|---|
| C1 | simulator speed | `rtf.csv` (or state rate/20 if missing) | RTF ≥ 0.85 | FAIL < 0.5 → **H1**: T5, T7 |
| C2 | control tick timing | `wall_dt_ms`, `step_ms` | < 2 % late ticks, step cost < 30 % of period | CPU starvation: T5 |
| C3 | ticks without a fresh state | `rx_since` | < 10 % stale | **H2**: T8 |
| C4 | predicted vs measured position | `pred_err` | p95 < 2 cm | stale ticks are drifting the model |
| C5 | drone follows velocity commands | fit `v = gain · lowpass(cmd delayed)` | gain 0.8–1.2, R² ≥ 0.8, total lag < 0.3 s (sim) | **H3**: T3a/T3b. Only meaningful with excitation (|cmd| rms ≥ 0.03); in a calm SPH run it says INFO |
| C6 | speed when commanded ≈ 0 | measured v on ticks with |cmd| < 1 cm/s for 2 s | < 10 mm/s | drone drifts by itself: **H3**, T2 |
| C7 | oscillation | spectrum of measured velocity, 0.3–8 Hz; ln-rms trend | not sustained | FAIL = growing → T4 |
| C8 | swarm shape and drift | ring radius trend, centroid speed, largest displacement | radius trend < 10 mm/s, drift < 10 mm/s, no drone moved > 15 cm | see C6, C15 |
| C9 | closest approach | min pair distance | > 0.25 m | safety |
| C10 | SPH density regime | rho / rho_0 | ≥ 80 % of the time within ±10 % | rho ≪ rho_0 = drones too far apart / neighbours missing |
| C11 | acceleration budget | rms of each term | INFO: which term actually moves the drones | |
| C12 | speed saturation | `clipped` | < 2 % | unclipped value keeps integrating on stale ticks |
| C13 | command chatter | sign flips between ticks | < 25 % | ζ·dt too large |
| C14 | altitude hold | z, vz | error < 5 cm, slope < 10 mm/s | vz = 0 is *not* an altitude hold |
| C15 | average command vs motion | mean cmd and mean v per drone | INFO; WARN if a drone is commanded > 10 mm/s on average but barely moves | the controller is holding against a constant push |

**Diagnosis block** (ranked HIGH → INFO): rule-based statements built from the checks, e.g. *TIME-BASE*, *STALE-STATE*,
*DRONE-TRACKING*, *OWN-DRIFT*, *GROWING-OSCILLATION*, *SATURATION*, *CHATTER*, *DESIGN* (ζ·dt ≥ 1), *GEOMETRY*
(ring radius ≠ equilibrium radius). They are leads, not proofs — each names the test that confirms it.

**Figures** (`analysis/`):

| File | Look for |
|---|---|
| `fig_overview.png` | six stacked panels on one time axis: speed, command, rho/rho_0, ring radius, stale %, z. Shaded = analysed window. Oscillation growth, drift and the startup transient are all visible here |
| `fig_tracking.png` | command (red) vs measured velocity (blue): lag, overshoot, gain, offset when the command is zero |
| `fig_spectrum.png` | velocity power vs frequency, wall axis and **sim** axis (÷ RTF). One sharp peak = a resonance; broad = noise |
| `fig_forces.png` | which acceleration term dominates |
| `fig_trajectory.png` | XY paths, equilibrium ring (green), potential circle |
| `fig_timing.png` | histograms of tick spacing, state-message spacing, messages per tick (the stale-tick picture) |

**Wall time vs sim time.** The controller and all logs use wall time. The drone physics uses sim time.
`sim = wall × RTF`; a frequency in sim Hz is `wall Hz / RTF`; a delay in sim seconds is `wall s × RTF`.
The analyzer prints both. Always compare runs at similar RTF (`C1`), or compare in sim time.

---

## 4. The test battery

Conventions: **defaults of every test = the setup of your analysed logs** — 5 drones, `POT_K=0` (potential force off),
`./dev_tmux.sh up-swarm`. Override on the command line, e.g. `tests/run_test.sh T1 N_DRONES=3 POT_K=2`.
Each run takes ≈ 2 min (15 s simulator + 15 s Crazyswarm + 20 s to takeoff + flight window + analysis).
Simulation is not deterministic: **run T1 twice** and compare before trusting small differences.
All commands are run from `~/SPH_Project`.

| Id | Command | Isolates | Time |
|---|---|---|---|
| T0 | `python3 src/offline_twin.py …` | instruments | 10 s |
| T1 | `tests/run_test.sh T1` | baseline, RTF, stale ticks | 2 min |
| T2 | `tests/run_test.sh T2` | the drone alone: hover drift | 2 min |
| T3a | `tests/run_test.sh T3a` | the drone alone: step response | 2 min |
| T3b | `tests/run_test.sh T3b` | the drone alone: bandwidth / resonance | 2.5 min |
| T4 | `tests/run_test.sh T4` | SPH loop gain sweep (4 runs) | 8 min |
| T5 | `tests/run_test.sh T5` | load → RTF (N and RViz; 6 runs) | 12 min |
| T7 | `tests/run_test.sh T7` | time-base: `DT_MODEL` sweep (4 runs) | 8 min |
| T13 | `tests/run_test.sh T13` | controller on simulated time, `SIM_CLOCK=gz` (the real fix for H1) | 4 min at RTF 1, ~8 min at RTF 0.12 |
| T8 | `tests/run_test.sh T8` | stale policy (3 runs) | 6 min |
| T9 | `tests/run_test.sh T9` | start gate (3 runs) | 6 min |
| T10 | `tests/run_test.sh T10` | scaling N = 3, 5, 7 (3 runs) | 6 min |
| T11 | `tests/run_test.sh T11` | external potential on/off (2 runs) | 4 min |

Batches: `tests/run_all.sh quick` (T1 T2 T3a, ≈ 8 min), `standard` (+ T3b T4 T8, ≈ 35 min), `full` (everything, ≈ 2 h).
A batch ends with a comparison table `src/logs/runs/compare_<batch>/compare.md`.

---

### T1 — Baseline

**Goal.** Reproduce your problem with the new instruments and measure the things the old logs could not: RTF,
state rate, stale ticks, measured vs commanded velocity.

**Run.** `tests/run_test.sh T1`

**Read.** `summary.txt`; `fig_overview.png`; `fig_timing.png`; `events.log` (`HB` lines show `rx=` Hz and `stale=` %).

| Observation | Means |
|---|---|
| C1 RTF ≈ 0.4 **and** state rate ≈ 8 Hz **and** stale ≈ 60 % | H1 + H2 confirmed as present: the state topic is sim-paced, the controller is not |
| RTF ≈ 0.4 but state rate ≈ 20 Hz, stale ≈ 0 % | state topic is wall-paced; H2 off, H1 still applies to the physics |
| RTF ≥ 0.85 | H1 and H2 are not your problem → T2, T3a, T4 |
| C7 FAIL with period `f_sim` | the oscillation is real; compare `f_sim` with the lag found in T3a (§6) |
| C6/C15: some drones need a constant command while barely moving | drift: T2 |

**Next.** Always T2 and T3a.

### T2 — Hover (zero command, SPH not in the loop)

**Goal.** Does the drone move by itself when told to stand still? `TEST_PROFILE=zero` replaces the SPH command with
`(0, 0)` after the start gate opens (all drones above 80 % of altitude for 1 s); SPH is still computed and logged, just not sent.

**Run.** `tests/run_test.sh T2`   (45 s of hover)

**Read.** `C6` (speed per drone while commanded 0), `C14` (altitude), `fig_tracking.png` (blue lines should sit on 0),
`fig_trajectory.png` (paths should be dots), RTF.

| Observation | Means |
|---|---|
| all drones < 10 mm/s, paths are dots | the drift in T1 is **not** the drone: it comes from the loop (T4, T7, T8) |
| some drones 20–50 mm/s, consistent direction (outward?) | **H3**: the drone/estimator/simulator drifts without SPH. Compare `meas_vx/vy` with the Gazebo pose to see whether the estimator or the physics moves (§7, optional T12) |
| all drones drift in the **same** direction | common estimator / world bias, not a ring effect |
| z slope ≠ 0 or z error > 5 cm | `vz = 0` is not an altitude hold (the README notes this for `send_velocity_world`): the firmware's own loop must hold z |
| velocity noise (v rms at hover) > 15 mm/s | estimator noise enters the loop: SPH will amplify it with `dt_model/RTF` |

### T3a — Step response of the drone

**Goal.** Identify the drone plant without SPH: gain, pure delay and time constant of "world velocity command → measured velocity".

**Run.** `tests/run_test.sh T3a`  (all drones get +0.2 m/s along x for 5 s, 3 s after the gate opens; the swarm moves ≈ 1 m as a block —
make sure there is room; `TEST_AXIS=y TEST_AMP=0.1` to change)

**Read.** `C5`, `fig_tracking.png` (red step, blue response). The analyzer prints **gain, delay (wall ms), tau (wall ms) and total lag in sim ms**.

| Observation | Means |
|---|---|
| gain 0.9–1.1, total lag < 150 ms (sim), R² > 0.9 | the drone is a good velocity actuator; H3 is not it |
| total lag 300–600 ms (sim) | slow plant. A loop around it oscillates near `f ≈ 1 / (4 · lag)` (e.g. 0.4 s → 0.6 Hz). Compare with `f_sim` of T1 |
| gain < 0.8 or > 1.2 | command is being scaled (clipping, firmware limits); controller loop gain differs from the design |
| delay ≫ tau | transport/stale delay: look at `hdr_lat_ms` and `rx_dt_ms` |
| R² < 0.5 | response not explained by a lag: saturation, mode switching (high-level ↔ low-level commander) or a crashed drone — look at the figure |
| clearly different gain/lag between drones | per-drone table in `report.md` (SITL instances differ?) |

### T3b — Chirp (frequency response)

**Goal.** Same plant, but excited from 0.05 → 1.5 Hz (wall) over 60 s: shows **where** tracking collapses and any resonance.

**Run.** `tests/run_test.sh T3b`

**Read.** `fig_tracking.png` (measured amplitude shrinking and lagging as the chirp speeds up), `fig_spectrum.png`, `C5` fit.
If the measured velocity is already < 50 % of the command at the oscillation frequency seen in T1, the loop cannot be stable at that frequency
with any gain > 1 there → reduce the SPH loop gain (T4) or slow the model (T7).

### T4 — SPH loop gain sweep (H4)

**Goal.** Find the critical SPH gain. `SPH_GAIN` multiplies **only** the pressure + viscosity acceleration; damping stays. 0 = pure damping.

**Run.** `tests/run_test.sh T4`  → runs `SPH_GAIN = 0, 0.25, 0.5, 1.0` (potential force off). Compare:
`python3 src/analyze_run.py src/logs/runs/*T4_gain* --compare --out src/logs/runs/compare_T4`

| Observation | Means |
|---|---|
| gain 0 calm (C7 PASS, C8 PASS) | damping alone is stable; the instability needs SPH |
| oscillation appears between gain a and b, growth rate rises with gain | classic loop-gain instability: the critical gain is the answer. Stiffness ∝ `particle_mass × gamma`; to move away from it lower mass/gamma, raise zeta (keep `zeta·dt < 1`), or slow the model (T7) |
| even gain 0 oscillates/drifts | not SPH: back to T2/T3 (drone) or T8 (stale handling) |
| oscillation at gain 1 only when RTF is low | H1: T7 |
| drift/ring expansion present at every gain including 0 | drone drift (T2) |

### T5 — Load and real-time factor (H1/H6)

**Goal.** What makes RTF drop? `N = 1, 2, 3` drones × RViz on/off. (`RVIZ=False` skips the GUI cost.)

**Run.** `tests/run_test.sh T5`   — compare `C1`, `C3` and C7 across runs.

| Observation | Means |
|---|---|
| RTF rises a lot with RViz off | RViz is stealing CPU → run with `RVIZ=False` (`swarm.conf`) |
| RTF falls steadily with N | simulator-bound; consider fewer drones per process, a faster machine, or Gazebo physics settings |
| RTF ≈ 1 for N = 1–2 and oscillation absent | the oscillation is a load effect → T7 |
| `LATE_TICK` / `SLOW_STEP` events | the controller itself is CPU-starved (C2) |

### T7 — Make the model match simulated time (H1 test and candidate fix)

**Goal.** `DT_MODEL` is the integration step of the SPH model per tick. With RTF = 0.4 one tick covers 20 ms of simulated time, so the
physically consistent value is `DT_MODEL = RTF × 0.05 = 0.02`. If H1 is the cause, shrinking `DT_MODEL` towards that value should shrink or remove the oscillation.

**Run.** `tests/run_test.sh T7`  → `DT_MODEL = 0.05, 0.03, 0.02, 0.0125`. Choose the list from the RTF measured in T1 (`RTF × 0.05` should be in it):
`DT_LIST="0.05 0.03 0.022 0.015" tests/run_test.sh T7`. A single value by hand: `DT_MODEL=0.02 RUN_LABEL=dt02 ./dev_tmux.sh up-swarm`.

| Observation | Means |
|---|---|
| growth rate falls monotonically with DT_MODEL; calm at ≈ RTF × 0.05 | **H1 confirmed.** Fix options: (a) run the controller on simulated time, (b) set `DT_MODEL` from the measured RTF, (c) raise RTF (T5) |
| no change | H1 is not the dominant factor |
| gets worse | the loop was limited by drone delay in *wall* time; combine with T8 |

*Note:* a smaller `DT_MODEL` also reduces the effective SPH gain and damping per tick by the same factor — that is the point, it restores the designed per-sim-second values. With `ζ·dt = 10 × 0.02 = 0.2` damping is gentler per tick; raise `SPH_ZETTA` if you need more.

### T13 — Controller on simulated time (the fix for H1)

**Goal.** Make the controller independent of the real-time factor. With `SIM_CLOCK=gz` the controller reads Gazebo's own clock
(`gz topic -e -t <world>/stats`, the same source as `rtf_logger.py`) and ticks **once every 50 ms of simulated time**. One tick then
always means `dt_model` of physics, at RTF 1.0, 0.4 or 0.12. Low RTF only makes the run slower. Test profiles and the `flight` gate hold
time are also counted in simulated seconds. Simulation only: on real drones the wall clock *is* the physical clock.

**Run.** `tests/run_test.sh T13` (5 drones, `START_GATE=flight`, 60 simulated seconds of flight; the script waits `SLOWDOWN`=4 times longer in
wall seconds, set `SLOWDOWN=8` if your RTF is about 0.12). By hand: `SIM_CLOCK=gz START_GATE=flight ./dev_tmux.sh up-swarm`.
Compare with T1 at the same N: `python3 src/analyze_run.py --compare <T1 run> <T13 run>`.

Check in the report: `clock=gz` in the params line, `TIME-BASE-OK` in the diagnosis, C2 `dt p50/p95/max = 50/50/50` (that is *simulated* ms now),
and in `events.log` the `SIMCLOCK_STOP` line (`missed_periods` should be about 0; many means the controller is CPU-starved even in sim time).

| Observation | Means |
|---|---|
| T13 calm where T1 oscillated or diverged | **H1 confirmed and fixed.** Keep `SIM_CLOCK=gz` for every simulation |
| T13 still oscillates at the same simulated frequency | not a time-base problem: go on with T3 (drone lag), T4 (SPH gain) |
| `SIMCLOCK_MISSING` error, drones never move | `gz` not found or no `*/stats` topic in the controller's environment: run `gz topic -l \| grep stats` in the container |
| `SIMCLOCK_STALL` warnings | the gz stats stream stopped for 2 s (Gazebo hung or died) |

Notes: (1) the drones still receive commands in wall time; to keep the firmware's setpoint watchdog fed while the sim is slow the controller
re-sends the last command every `KEEPALIVE_S` = 0.1 wall seconds (no re-integration). (2) The first ~15 s are slow at low RTF because takeoff is simulated;
`START_GATE=flight` hides that. (3) At very low RTF the state topic still arrives, so each tick uses the latest state: that is fine.

### T8 — Stale-tick policy (H2)

**Goal.** What the controller does on a tick with no new state message: `predict` (current: integrate the prediction again), `hold`
(re-send the previous command, no integration), `skip` (publish nothing).

**Run.** `tests/run_test.sh T8`

| Observation | Means |
|---|---|
| `hold` or `skip` calmer than `predict` | prediction on stale ticks injects error (C4) → keep `STALE_POLICY=hold` |
| no difference | stale ticks are not the issue |
| `skip` worse | the firmware needs a continuous setpoint stream (watchdog) |

### T9 — Start gate (H5)

**Goal.** `START_GATE=none` (current) lets SPH command from the first received state — on the ground, with only some drones known.
`all_known` waits for every drone; `flight` waits until all are above 80 % of the target altitude for 1 s.

**Run.** `tests/run_test.sh T9`   — read `fig_overview.png` (rho panel before takeoff) and `RHO_LOW` events.

| Observation | Means |
|---|---|
| `none`: rho ≈ 550 and large |v| in the first seconds; `flight`: clean start | the startup transient kicks the swarm; use `START_GATE=flight` |
| identical after takeoff | the transient is harmless after 10 s |

### T10 — Scaling with N (H6)

`tests/run_test.sh T10` → N = 3, 5, 7 (spawn ring grows with N so the density stays rho_0). Compare RTF, growth rate, `C8`.
Oscillation onset between N values + falling RTF → H1; onset without RTF change → SPH geometry/neighbour count.

### T11 — External potential (H7)

`tests/run_test.sh T11` → `POT_K = 0` and `2`. Note `swarm_sph` now **honours** `POT_K` (before, the force line was commented out).
Compare `C11` (the external term in the acceleration budget), C8 (does the ring converge to the circle?) and C7.

### T12 (optional, manual) — Ground truth vs estimator

The logs contain the **firmware's estimate** (`stateEstimate.*`), not Gazebo's pose. If T2 shows drift, check whether the drone really moves:
in a container shell `gz topic -l | grep -i pose`, then `gz topic -e -t <world>/dynamic_pose/info` while hovering, and compare to `meas_x/y` in
`tick_<drone>.csv`. Drift in the estimate only → estimator (firmware params in `crazyflies.yaml`); drift in both → physics/controller.

---

## 5. Decision tree

```
T1: is RTF < 0.85 ?
 ├─ yes → T5 (can you raise it? RVIZ=False, fewer drones)         -- H1/H2 present
 │         T7 (DT_MODEL = RTF*0.05)  → oscillation gone?  → time-base was the cause
 │         T8 (hold)                  → calmer?           → keep STALE_POLICY=hold
 └─ no  → continue

T2: do drones move at hover (> 10 mm/s)?
 ├─ yes → drone/estimator/simulator drift (T12 to see which). SPH cannot be blamed for that part.
 └─ no  → continue

T3a: gain 0.8–1.2, total lag (sim) < 0.3 s, R2 > 0.8 ?
 ├─ no  → the drone is a slow / non-ideal actuator. Fix the plant side, or lower the SPH loop gain (T4)
 └─ yes → continue

T4: gain sweep: oscillation only above some gain?
 ├─ yes → the SPH loop is too stiff for this plant: lower mass/gamma, raise zeta, slow the model (T7)
 └─ no  → the problem is not in the loop's linear behaviour: look at C13 (chatter), C12 (saturation), T9 (startup)

Still unexplained → send me summary.txt of T1, T2, T3a and compare.md
```

---

## 6. Interpretation cheat sheet

| Quantity | Formula | Use |
|---|---|---|
| expected stale ticks (sim-paced state) | `1 − RTF · state_hz / tick_hz` = `1 − RTF` at 20/20 Hz | compare with C3 |
| controller gain per simulated second vs design | `dt_model / (RTF · period)` | 0.05 / (0.4·0.05) = 2.5× |
| model-consistent step | `DT_MODEL = RTF · period` | T7 |
| frequency in sim Hz | `f_wall / RTF` | C7 prints both |
| lag in sim s | `lag_wall · RTF` | C5 prints both |
| delay-limited oscillation | `f ≈ 1 / (4 · L)`, L = total lag (sim s) | match to C7 `f_sim` |
| damping per tick | `1 − ζ · dt_model` | must stay in (0, 1): `ζ·dt < 1` or the command reverses each tick (C13) |
| ring equilibrium radius | where SPH density = rho_0 (printed in C8) | `GEOMETRY` diagnosis if spawn ≠ equilibrium |

**Healthy reference** (the offline twin with an ideal drone at RTF 1): C1–C14 all PASS, velocity rms ≈ 0.01 m/s (estimator noise),
radius trend ≈ 0, centroid drift ≈ 0, stale ticks ≈ 0 %.

**Blind spots:** no ground truth (see T12); the analysis of C5 in a closed-loop SPH run is biased (use T3); velocity noise
`σ ≈ 10 mm/s` limits what C6 can see; RTF is sampled at 4 Hz.

---

## 7. What to send me

1. `cat src/logs/runs/<id>/analysis/summary.txt` for each test you ran (they are short).
2. `src/logs/runs/compare_*/compare.md` if you ran a batch or a sweep.
3. If something looks strange: `head -60 src/logs/runs/<id>/events.log` and the figure(s) named in the diagnosis.
4. If the analyzer itself misbehaves: the traceback and `ls -la src/logs/runs/<id>`.

Tell me which test ids and any command-line overrides you used (they are also stored in `meta.json`).

---

## 8. Troubleshooting the instruments

| Symptom | Cause / fix |
|---|---|
| `rtf.csv` missing, C1 says "estimated from state rate" | the `rtf` tmux window could not find a `*/stats` topic: in the container run `gz topic -l \| grep stats`; if it has another name run `python3 src/tools/rtf_logger.py --topic <name> --out <file>`. The estimate (state rate / 20) is valid only if the state topic is sim-paced |
| `run folder … not found` on `analyze` | `cat .last_run_id`, `ls src/logs/runs`; run `python3 src/analyze_run.py src/logs/runs/<id>` directly |
| analysis cannot write (`Permission denied`) | the run folder is root-owned: `./dev_tmux.sh stop` fixes ownership; or `sudo chown -R $USER src/logs` |
| `GATE_WAIT` forever in `events.log` | a drone never reported (name in the message) or never climbed above 0.8·`FIXED_Z`: is `AUTO_TAKEOFF=1`? did the takeoff service succeed? |
| report says "NO FLIGHT DETECTED – whole run" | drones never reached 80 % of `fixed_z` (z from the estimator): takeoff failed or `FIXED_Z` mismatches the takeoff height |
| several `…_run` folders from one launch | launch was started without `run_id` (e.g. by hand); `dev_tmux.sh` always passes one |
| `ros2 launch … 'xyz' is not a valid argument` | the container still has the old launch files: `colcon build --symlink-install` (done automatically by `up`) |
| report is mostly `N/A` | you analysed old `plain_sph_*.csv` files (only 9 columns); rerun with the new code |

---

## Appendix A — all switches

| `swarm.conf` / env | launch arg = ROS param | default | Effect |
|---|---|---|---|
| `RUN_LABEL` | `run_label` (+ `run_id`, `git_rev` set by `dev_tmux.sh`) | `run` | name of the run folder |
| `SPH_GAIN` | `sph_gain` | 1.0 | multiplies pressure + viscosity acceleration; 0 = off |
| `DT_MODEL` | `dt_model` | 0.0 (= period) | SPH integration step per tick (s) |
| `CONTROL_PERIOD` | `control_period` | 0.05 | tick period: wall s (`SIM_CLOCK=wall`) or simulated s (`gz`) |
| `SIM_CLOCK` | `sim_clock` | `wall` | `gz` = tick on Gazebo simulated time (simulation only) |
| `MAX_VELOCITY` | `max_velocity` | 0.5 | speed limit of the command, m/s |
| — | `keepalive_s`, `poll_period`, `gz_stats_topic` | 0.1, 0.005, auto | sim clock: command re-send interval (wall s), clock poll interval, stats topic |
| `STALE_POLICY` | `stale_policy` | `predict` | `predict` \| `hold` \| `skip` on ticks with no new state |
| `START_GATE` | `start_gate` | `none` | `none` \| `all_known` \| `flight` |
| `TEST_PROFILE` | `test_profile` | `none` | `zero` \| `step` \| `square` \| `sine` \| `chirp` replace the SPH command |
| `TEST_AMP` `TEST_PERIOD` `TEST_AXIS` `TEST_DELAY` `TEST_DURATION` | `test_*` | 0.15, 4, x, 3, 60 | profile shape (m/s, s, axis, s after gate, s for chirp) |
| `POT_K` | `f_k` | 2.0 (tests: 0) | potential force gain; now also honoured by `swarm_sph` |
| `DETACH=1` | — | 0 | `dev_tmux.sh up…` does not attach to tmux |

Test profiles: `zero` 0 m/s; `step` 0 for `delay`, then +amp for `period` s; `square` ±amp, cycle = `period`; `sine` amp·sin(2πt/period);
`chirp` 0.05 → 1.5 Hz (log sweep) over `duration`. Same command to every drone, along `axis`; time zero = gate opens, so use `START_GATE=flight`.

## Appendix B — design notes (why the logs look the way they do)

* **Two files per drone, not one**: `rx_*` is what the drone sent (independent of controller timing), `tick_*` is what the controller did. Their difference
  (`rx_since`, `state_age_ms`, `pred_err`) is the stale-state story.
* **`meas_*` vs `used_*`**: on a fresh tick they are equal; on a stale tick `used_*` is the controller's own prediction. The old logs wrote only the latter.
* **The acceleration budget is logged, not recomputed**: `ap + av + ad + ae` = SPH part (× `sph_gain`) + damping + external = `a`.
* **`epoch` in every row** lets `rtf.csv` (written by another process) be aligned to ticks.
* **Logging cannot stop the controller**: all file errors are swallowed after one warning; WARNs are rate-limited.
* `SphCore` (`sph_core.py`) holds the control loop and is ROS-free; `plain_sph.py` / `swarm_sph.py` only connect it to ROS.
  `tests/test_core_equivalence.py` checks it reproduces the original loops to 1e-9 on random swarms with missing messages.

## Appendix C — files changed or added

Changed: `particle.py` (force split + `sph_gain`; `next_vel` numerically identical), `plain_sph.py`, `swarm_sph.py` (thin wrappers),
`launch/plain_sph.launch.py`, `launch/swarm_sph.launch.py`, `dev_tmux.sh`, `swarm.conf`, `README.MD`, `.gitignore`.
Added: `drone_controller/sph_core.py`, `sph_diag.py`, `sph_ros.py`; `src/analyze_run.py`, `src/offline_twin.py`, `src/tools/rtf_logger.py`;
`tests/run_test.sh`, `tests/run_all.sh`, `tests/test_core_equivalence.py`, `tests/test_analyzer.py`; this guide.
**Not run here:** Gazebo, ROS and Docker are not available where this was written, so the ROS wrappers (`sph_ros.py`, `plain_sph.py`,
`swarm_sph.py`), `dev_tmux.sh`, the launch files and `rtf_logger.py` against a live `gz` were only compile/syntax/stub-checked.
The control loop, logger, analyzer and runner scripts were exercised offline (twin, synthetic runs, stub launcher). The smoke test in §2 is the first thing to run.

**Added later (simulated-time controller, real-drone support):** `drone_controller/sim_clock.py` (Gazebo clock + tick scheduler), `tests/test_sim_clock.py`,
`REAL_DRONES.md`, `config/crazyflies_real.example.yaml`; changed `sph_core.py` (`step(now, sim_t, sim_dt)`, safety latch, `max_velocity`), `sph_ros.py` (`ControlDriver`),
both wrappers (+ `~/hover` service), launch files, `package.xml` (std_srvs), `swarm.conf`, `dev_tmux.sh`, `run_test.sh` (T13), `offline_twin.py` (`--sim_clock gz`), `analyze_run.py`.
**Also not run against a live Gazebo:** the `gz topic -e` reader was tested against a fake stream in the real text format (`tests/test_sim_clock.py`), not against `gz` itself.
