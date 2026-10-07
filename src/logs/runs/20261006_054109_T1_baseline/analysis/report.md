# SPH run report: 20261006_054109_T1_baseline

- run: 20261006_054109_T1_baseline   label: T1_baseline   code: fc36220-dirty   drones: 5   mode: swarm
- window analysed: t = 4.9 .. 56.0 s (after flight start at 2.9 s)
- params: sph_gain=1.0 zeta=10.0 gamma=2.0 mass=7000.0 f_k=0.0 dt_model=0.05 policy=predict gate=none

## Diagnosis

- **[HIGH] TIME-BASE** - The simulator runs at about 0.12x real time while the controller ticks at 20 Hz of WALL time: each control tick covers only 6 ms of simulated time instead of 50 ms, and the 20 Hz state topic arrives at about 16.2 Hz wall. The SPH model integrates dt=50 ms per tick, but only 6 ms of simulated time passed, so per SIMULATED second the controller applies 8.5x the acceleration the SPH model intends. Test T6/T7 (RTF sweep, dt_model scaled by RTF).
- **[HIGH] DRONE-TRACKING** - The drone does not follow velocity commands like an ideal integrator: gain -373.58, total lag 75 ms (sim), R2 0.09. Any controller closed around this has less phase margin; compare with T3 (step/chirp test, no SPH).
- **[HIGH] GROWING-OSCILLATION** - Velocity amplitude grows (+0.131 per s on ln rms) at 4.46 Hz sim (0.53 Hz wall): the loop is unstable. Delay-induced instability would oscillate near 1/(4 x lag) = 3.22 Hz sim; observed 4.46 Hz sim. Find which part causes it with T1 (damping only) -> T4 (SPH only).
- **[MED] STALE-STATE** - 23 % of ticks have no new state message; with stale_policy=predict those ticks integrate the controller's own prediction. Compare stale_policy=hold (T8).
- **[MED] SATURATION** - 13 % of ticks hit the speed limit.

## Health checks

| id | check | result | measured | threshold / meaning |
|---|---|---|---|---|
| C1 | simulator speed (real-time factor) | **FAIL** | RTF 0.12 (p5 0.04), source rtf.csv | >= 0.85 ok; < 0.5 fail. The controller ticks in WALL time; the drones live in SIM time. |
| C2 | control tick timing | **WARN** | dt p50/p95/max = 50/70/336 ms (nominal 50); late 3.5 %; step cost p95 20.17 ms | late > 2.0 % warn, > 10.0 % fail; step cost should stay < 30 % of the period |
| C3 | ticks without a fresh state message | **WARN** | 22.9 % stale (longest run 5 ticks), 6.5 % ticks got 2+ messages; stale_policy=predict | < 10.0 % ok. With policy "predict" a stale tick integrates the controller's own prediction again |
| C4 | predicted vs measured position | **WARN** | p95 gap 3.1 cm | < 2 cm ok |
| C5 | drone follows velocity commands | **FAIL** | model v = gain * lowpass(cmd delayed): gain -373.58, delay 400 ms, tau 232 ms (wall) => total lag 75 ms of sim time, R2 0.09 (closed loop: SPH is driving, estimate is biased; run a test profile for a clean one) | gain 0.8-1.2, R2 >= 0.8, sim-time lag < 0.3 s |
| C6 | drone speed when commanded ~0 (own drift) | **PASS** | cf_0: 1 mm/s, cf_2: 1 mm/s, cf_4: 1 mm/s | < 10 mm/s ok. Non-zero = the drone/estimator moves by itself |
| C7 | oscillation of the drones | **FAIL** | oscillation (>0.3 Hz) rms 562.009 m/s, dominant 0.53 Hz wall = 4.46 Hz sim (peak 64x median), amplitude trend 0.131 /s -> GROWING; slow part (<0.3 Hz, drift) rms 220.630 m/s | sustained = peak > 6.0x median AND oscillation rms > 0.02 m/s; growing = ln(rms) slope > 0.03/s |
| C8 | swarm shape and drift | **FAIL** | mean radius from centroid 1.51 -> 2.62 m over 51 s (trend +21.8 mm/s); equilibrium (rho=rho0) ring radius 1.51 m; centroid moved 142 cm (27.7 mm/s); largest displacement 4.35 m (cf_2) | radius trend and centroid speed should be ~0 when the swarm is at rest |
| C9 | closest approach between drones | **FAIL** | min 0.00 m, median 1.66 m | > 0.25 m ok (Crazyflie ~0.1 m across, downwash) |
| C10 | SPH density regime | **WARN** | rho within +-10 % of rho0 52 % of the window; rho 354..1279, P -7698..16302; first within 20 % of rho0 at t=2.9 s | >= 80 % in band. Large |P| means SPH is pushing hard; rho << rho0 = drones too far apart or neighbours missing |
| C11 | acceleration budget (rms, m/s^2) | **INFO** | pressure 2.543, viscosity 34284510.304, damping 11608772.815, external 0.000, total 36322175.730 -> dominant: viscosity | tells which term actually moves the drones |
| C12 | speed command saturation | **FAIL** | 13.2 % of ticks clipped at 0.5 m/s | < 2.0 % ok. The un-clipped value keeps integrating in stale ticks |
| C13 | command chatter (direction flips tick to tick) | **PASS** | 11 % of consecutive ticks reverse direction | < 25.0 % ok. High = damping gain too big for the step (zeta * dt > 1) |
| C14 | altitude hold (z, commanded vz = 0) | **FAIL** | z mean 1.65 m (target 0.5), worst error 167 cm, worst slope 55.0 mm/s | < 5 cm error and < 10 mm/s slope |
| C15 | average command vs average motion | **INFO** | cf_0: cmd 100 mm/s (radial +61), moves 149 mm/s; cf_1: cmd 20 mm/s (radial -18), moves 262465 mm/s; cf_2: cmd 40 mm/s (radial -14), moves 53 mm/s; cf_3: cmd 57 mm/s (radial +48), moves 64 mm/s; cf_4: cmd 39 mm/s (radial +30), moves 72637 mm/s | a persistent command with almost no motion = the controller is holding against a constant push (drone drift, potential force, or an SPH imbalance) |

## Per drone

| drone | rx Hz | stale % | pred err p95 m | gain | delay ms | tau ms | R2 | osc rms m/s | slow rms m/s | osc Hz sim | rms slope /s | rho mean | P mean | a press | a damp | a ext | moved m | hover vx | hover vy | z | flip % |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| cf_0 | 16.3 | 22 | 0.019 | 1.42 | 0 | 20 | 0.12 | 0.844 | 0.316 | 5.00 | 0.078 | 869 | -3375 | 2.380 | 27631418.890 | 0.000 | 4.01 | -0.001 | -0.000 | 2.17 | 6 |
| cf_1 | 16.2 | 24 | 0.031 | -958.26 | 1000 | 400 | 0.00 | 2279.262 | 796.442 | 4.46 | 0.221 | 896 | -2871 | 2.058 | 29757605.226 | 0.000 | 1.03 | n/a | n/a | 1.69 | 15 |
| cf_2 | 16.4 | 23 | 0.018 | 1.46 | 0 | 20 | 0.19 | 0.676 | 0.230 | 5.49 | 0.079 | 862 | -3384 | 2.720 | 147345.949 | 0.000 | 4.35 | 0.001 | 0.001 | 1.72 | 14 |
| cf_3 | 16.2 | 22 | 0.016 | 1.22 | 0 | 20 | 0.09 | 0.821 | 0.182 | 4.05 | 0.035 | 826 | -4260 | 2.708 | 53132.645 | 0.000 | 0.51 | n/a | n/a | 1.10 | 8 |
| cf_4 | 16.1 | 23 | 0.016 | -913.76 | 1000 | 700 | 0.02 | 528.442 | 305.981 | 3.43 | 0.243 | 851 | -3396 | 2.851 | 454361.365 | 0.000 | 1.09 | 0.001 | -0.001 | 1.54 | 10 |

## Events (events.log)

| level | code | count |
|---|---|---|
| WARN | LATE_TICK | 45 |
| WARN | CLIP_START | 28 |
| WARN | RHO_LOW | 24 |
| WARN | PRED_ERR | 18 |
| WARN | SLOW_STEP | 15 |
| WARN | STALE_RUN | 11 |
| WARN | RHO_HIGH | 1 |
| INFO | HB | 55 |
| INFO | CLIP_END | 28 |
| INFO | RHO_OK | 26 |
| INFO | FIRST_RX | 5 |
| INFO | START | 1 |
| INFO | STOP | 1 |

## Figures

- `fig_forces.png`
- `fig_overview.png`
- `fig_spectrum.png`
- `fig_timing.png`
- `fig_tracking.png`
- `fig_trajectory.png`

## Glossary

- **stale tick**: the control timer fired but no new state message from that drone arrived since the previous tick.
- **RTF**: Gazebo real-time factor = simulated seconds per wall second. Control ticks use wall time.
- **hover drift**: mean measured velocity on ticks where the command was ~0.
- **tau / delay**: first-order time constant and pure delay fitted between command and measured velocity (wall seconds; multiplied by RTF to get sim seconds).
- **a press / damp / ext**: rms of the pressure, damping (-zeta v) and external-force parts of dv/dt.
- full column list of the raw logs: see sph_diag.py (TICK_COLUMNS) or TEST_GUIDE.md section 3.
