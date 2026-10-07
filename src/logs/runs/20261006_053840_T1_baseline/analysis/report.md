# SPH run report: 20261006_053840_T1_baseline

- run: 20261006_053840_T1_baseline   label: T1_baseline   code: fc36220-dirty   drones: 2   mode: swarm
- window analysed: t = 9.2 .. 28.2 s (after flight start at 7.2 s)
- params: sph_gain=1.0 zeta=10.0 gamma=2.0 mass=7000.0 f_k=0.0 dt_model=0.05 policy=predict gate=none

## Diagnosis

- **[MED] TIME-BASE** - The simulator runs at about 0.49x real time while the controller ticks at 20 Hz of WALL time: each control tick covers only 25 ms of simulated time instead of 50 ms, and the 20 Hz state topic arrives at about 19.3 Hz wall. The SPH model integrates dt=50 ms per tick, but only 25 ms of simulated time passed, so per SIMULATED second the controller applies 2.0x the acceleration the SPH model intends. Test T6/T7 (RTF sweep, dt_model scaled by RTF).

## Health checks

| id | check | result | measured | threshold / meaning |
|---|---|---|---|---|
| C1 | simulator speed (real-time factor) | **FAIL** | RTF 0.49 (p5 0.29), source rtf.csv | >= 0.85 ok; < 0.5 fail. The controller ticks in WALL time; the drones live in SIM time. |
| C2 | control tick timing | **PASS** | dt p50/p95/max = 50/51/59 ms (nominal 50); late 0.0 %; step cost p95 2.22 ms | late > 2.0 % warn, > 10.0 % fail; step cost should stay < 30 % of the period |
| C3 | ticks without a fresh state message | **PASS** | 4.6 % stale (longest run 1 ticks), 1.2 % ticks got 2+ messages; stale_policy=predict | < 10.0 % ok. With policy "predict" a stale tick integrates the controller's own prediction again |
| C4 | predicted vs measured position | **PASS** | p95 gap 0.0 cm | < 2 cm ok |
| C5 | drone follows velocity commands | **INFO** | command rms only 0.017 m/s: too small to identify the drone model (this is normal for a calm SPH run). Run test T3 (step/chirp) for gain and lag. | needs |cmd| rms >= 0.03 m/s |
| C6 | drone speed when commanded ~0 (own drift) | **INFO** | no stretch with |cmd| < 1 cm/s for 2 s in the window; run test T2 (zero command) |  |
| C7 | oscillation of the drones | **PASS** | oscillation (>0.3 Hz) rms 0.001 m/s, dominant 0.42 Hz wall = 0.86 Hz sim (peak 184x median), amplitude trend 0.075 /s -> none/steady; slow part (<0.3 Hz, drift) rms 0.015 m/s | sustained = peak > 6.0x median AND oscillation rms > 0.02 m/s; growing = ln(rms) slope > 0.03/s |
| C8 | swarm shape and drift | **PASS** | mean radius from centroid 0.05 -> 0.05 m over 19 s (trend +0.0 mm/s); equilibrium (rho=rho0) ring radius n/a m; centroid moved 2 cm (0.9 mm/s); largest displacement 0.02 m (cf_0) | radius trend and centroid speed should be ~0 when the swarm is at rest |
| C9 | closest approach between drones | **FAIL** | min 0.11 m, median 0.11 m | > 0.25 m ok (Crazyflie ~0.1 m across, downwash) |
| C10 | SPH density regime | **WARN** | rho within +-10 % of rho0 0 % of the window; rho 707..707, P -7075..-7075 | >= 80 % in band. Large |P| means SPH is pushing hard; rho << rho0 = drones too far apart or neighbours missing |
| C11 | acceleration budget (rms, m/s^2) | **INFO** | pressure 0.352, viscosity 0.008, damping 0.152, external 0.000, total 0.198 -> dominant: pressure | tells which term actually moves the drones |
| C12 | speed command saturation | **PASS** | 0.0 % of ticks clipped at 0.5 m/s | < 2.0 % ok. The un-clipped value keeps integrating in stale ticks |
| C13 | command chatter (direction flips tick to tick) | **PASS** | 0 % of consecutive ticks reverse direction | < 25.0 % ok. High = damping gain too big for the step (zeta * dt > 1) |
| C14 | altitude hold (z, commanded vz = 0) | **PASS** | z mean 0.45 m (target 0.5), worst error 5 cm, worst slope 0.9 mm/s | < 5 cm error and < 10 mm/s slope |
| C15 | average command vs average motion | **INFO** | cf_0: cmd 25 mm/s (radial -25), moves 15 mm/s; cf_1: cmd 24 mm/s (radial -24), moves 14 mm/s | a persistent command with almost no motion = the controller is holding against a constant push (drone drift, potential force, or an SPH imbalance) |

## Per drone

| drone | rx Hz | stale % | pred err p95 m | osc rms m/s | slow rms m/s | osc Hz sim | rms slope /s | rho mean | P mean | a press | a damp | a ext | moved m | z | flip % |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| cf_0 | 19.3 | 4 | 0.000 | 0.002 | 0.015 | 0.83 | 0.082 | 707 | -7075 | 0.352 | 0.158 | 0.000 | 0.02 | 0.45 | 0 |
| cf_1 | 19.3 | 5 | 0.000 | 0.001 | 0.014 | 0.89 | 0.067 | 707 | -7075 | 0.352 | 0.146 | 0.000 | 0.02 | 0.45 | 0 |

## Events (events.log)

| level | code | count |
|---|---|---|
| INFO | HB | 10 |
| INFO | FIRST_RX | 2 |
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
