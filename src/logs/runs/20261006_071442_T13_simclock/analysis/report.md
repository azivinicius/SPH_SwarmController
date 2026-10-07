# SPH run report: 20261006_071442_T13_simclock

- run: 20261006_071442_T13_simclock   label: T13_simclock   code: fc36220-dirty   drones: 5   mode: swarm
- window analysed: t = 4.5 .. 17.5 s (after flight start at 2.5 s)
- params: sph_gain=1.0 zeta=10.0 gamma=2.0 mass=7000.0 f_k=0.0 dt_model=0.05 policy=predict gate=flight clock=wall

## Diagnosis

- **[HIGH] DIVERGED** - The run blew up at t = 18.0 s (cf_0: |v| = 0.0 m/s, z = 1.50 m; physically impossible for a drone commanded <= 0.5 m/s: crash or estimator divergence). Numbers below use only t < 17.5 s. Look at fig_overview.png and the first WARN lines of events.log before that time: that is where the cause is.
- **[MED] TIME-BASE** - The simulator runs at about 0.12x real time while the controller ticks at 20 Hz of WALL time: each control tick covers only 6 ms of simulated time instead of 50 ms, and the 20 Hz state topic arrives at about 15.9 Hz wall. The SPH model integrates dt=50 ms per tick, but only 6 ms of simulated time passed, so per SIMULATED second the controller applies 8.1x the acceleration the SPH model intends. Test T6/T7 (RTF sweep, dt_model scaled by RTF).
- **[MED] STALE-STATE** - 25 % of ticks have no new state message; with stale_policy=predict those ticks integrate the controller's own prediction. Compare stale_policy=hold (T8).

## Health checks

| id | check | result | measured | threshold / meaning |
|---|---|---|---|---|
| C1 | simulator speed (real-time factor) | **FAIL** | RTF 0.12 (p5 0.05), source rtf.csv | >= 0.85 ok; < 0.5 fail. The controller ticks in WALL time; the drones live in SIM time. |
| C2 | control tick timing | **WARN** | dt p50/p95/max = 50/69/120 ms (nominal 50); late 3.6 %; step cost p95 18.05 ms | late > 2.0 % warn, > 10.0 % fail; step cost should stay < 30 % of the period |
| C3 | ticks without a fresh state message | **WARN** | 25.2 % stale (longest run 8 ticks), 6.4 % ticks got 2+ messages; stale_policy=predict | < 10.0 % ok. With policy "predict" a stale tick integrates the controller's own prediction again |
| C4 | predicted vs measured position | **PASS** | p95 gap 0.0 cm | < 2 cm ok |
| C5 | drone follows velocity commands | **INFO** | command rms only 0.002 m/s: too small to identify the drone model (this is normal for a calm SPH run). Run test T3 (step/chirp) for gain and lag. | needs |cmd| rms >= 0.03 m/s |
| C6 | drone speed when commanded ~0 (own drift) | **PASS** | cf_0: 0 mm/s, cf_1: 1 mm/s, cf_2: 0 mm/s, cf_3: 1 mm/s, cf_4: 0 mm/s | < 10 mm/s ok. Non-zero = the drone/estimator moves by itself |
| C7 | oscillation of the drones | **PASS** | oscillation (>0.3 Hz) rms 0.009 m/s, dominant 0.79 Hz wall = 6.39 Hz sim (peak 551x median), amplitude trend n/a /s -> none/steady; slow part (<0.3 Hz, drift) rms 0.003 m/s | sustained = peak > 6.0x median AND oscillation rms > 0.02 m/s; growing = ln(rms) slope > 0.03/s |
| C8 | swarm shape and drift | **PASS** | mean radius from centroid 1.51 -> 1.51 m over 13 s (trend -0.2 mm/s); equilibrium (rho=rho0) ring radius 1.51 m; centroid moved 0 cm (0.2 mm/s); largest displacement 0.01 m (cf_3) | radius trend and centroid speed should be ~0 when the swarm is at rest |
| C9 | closest approach between drones | **PASS** | min 1.76 m, median 1.77 m | > 0.25 m ok (Crazyflie ~0.1 m across, downwash) |
| C10 | SPH density regime | **PASS** | rho within +-10 % of rho0 100 % of the window; rho 999..1005, P -26..181; first within 20 % of rho0 at t=2.5 s | >= 80 % in band. Large |P| means SPH is pushing hard; rho << rho0 = drones too far apart or neighbours missing |
| C11 | acceleration budget (rms, m/s^2) | **INFO** | pressure 0.013, viscosity 0.014, damping 0.076, external 0.000, total 0.087 -> dominant: damping | tells which term actually moves the drones |
| C12 | speed command saturation | **PASS** | 0.0 % of ticks clipped at 0.5 m/s | < 2.0 % ok. The un-clipped value keeps integrating in stale ticks |
| C13 | command chatter (direction flips tick to tick) | **PASS** | 0 % of consecutive ticks reverse direction | < 25.0 % ok. High = damping gain too big for the step (zeta * dt > 1) |
| C14 | altitude hold (z, commanded vz = 0) | **FAIL** | z mean 1.24 m (target 0.5), worst error 79 cm, worst slope 34.2 mm/s | < 5 cm error and < 10 mm/s slope |
| C15 | average command vs average motion | **INFO** | cf_0: cmd 0 mm/s (radial +0), moves 0 mm/s; cf_1: cmd 1 mm/s (radial +1), moves 1 mm/s; cf_2: cmd 1 mm/s (radial +0), moves 1 mm/s; cf_3: cmd 1 mm/s (radial +1), moves 1 mm/s; cf_4: cmd 0 mm/s (radial +0), moves 1 mm/s | a persistent command with almost no motion = the controller is holding against a constant push (drone drift, potential force, or an SPH imbalance) |

## Per drone

| drone | rx Hz | stale % | pred err p95 m | osc rms m/s | slow rms m/s | osc Hz sim | rms slope /s | rho mean | P mean | a press | a damp | a ext | moved m | hover vx | hover vy | z | flip % |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| cf_0 | 15.9 | 26 | 0.000 | 0.003 | 0.001 | 6.08 | n/a | 1000 | -3 | 0.010 | 0.026 | 0.000 | 0.00 | -0.000 | -0.000 | 1.29 | 0 |
| cf_1 | 16.1 | 25 | 0.000 | 0.009 | 0.002 | 6.39 | n/a | 1000 | -4 | 0.009 | 0.086 | 0.000 | 0.00 | -0.000 | 0.000 | 1.23 | 0 |
| cf_2 | 15.7 | 24 | 0.000 | 0.005 | 0.001 | 6.24 | n/a | 1000 | 5 | 0.013 | 0.044 | 0.000 | 0.00 | 0.000 | 0.000 | 1.18 | 0 |
| cf_3 | 15.4 | 28 | 0.000 | 0.019 | 0.005 | 7.58 | n/a | 1000 | 18 | 0.019 | 0.162 | 0.000 | 0.01 | -0.001 | -0.001 | 1.26 | 0 |
| cf_4 | 16.4 | 24 | 0.000 | 0.008 | 0.003 | 7.10 | n/a | 1000 | 3 | 0.012 | 0.062 | 0.000 | 0.00 | -0.000 | 0.000 | 1.24 | 0 |

## Events (events.log)

| level | code | count |
|---|---|---|
| WARN | LATE_TICK | 130 |
| WARN | RHO_LOW | 126 |
| WARN | CLIP_START | 110 |
| WARN | PRED_ERR | 99 |
| WARN | RHO_HIGH | 96 |
| WARN | STALE_RUN | 68 |
| WARN | SLOW_STEP | 57 |
| INFO | HB | 165 |
| INFO | RHO_OK | 121 |
| INFO | CLIP_END | 110 |
| INFO | FIRST_RX | 5 |
| INFO | START | 1 |
| INFO | GATE_WAIT | 1 |
| INFO | GATE_OPEN | 1 |
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
