# hummingbird-datacollec-pipe
Simulation-based quadrotor flight data collection pipeline. Generates CSV datasets from RotorS/Gazebo using diverse trajectory primitives, yaw modes, and wind disturbances.

# quad-koopman-datapipe

**Simulation-based quadrotor flight data collection pipeline for structured deep Koopman dynamics modeling.**

Generates NeuroBEM-compatible CSV datasets from RotorS/Gazebo simulations of the Hummingbird quadrotor, with diverse trajectory primitives, four yaw modes, and optional wind disturbances. Designed for training Koopman operator models of quadrotor dynamics.

---

## Table of Contents

1. [Repository Layout](#1-repository-layout)
2. [Pipeline Overview](#2-pipeline-overview)
3. [ROS Nodes and Topics](#3-ros-nodes-and-topics)
4. [Dataset Schema](#4-dataset-schema)
5. [Dataset Statistics (v10)](#5-dataset-statistics-v10)
6. [Trajectory Primitives](#6-trajectory-primitives)
7. [Yaw Modes](#7-yaw-modes)
8. [Wind Disturbance Model](#8-wind-disturbance-model)
9. [PI Wrapper](#9-pi-wrapper)
10. [Vehicle Parameters](#10-vehicle-parameters)
11. [Koopman State Layout](#11-koopman-state-layout)
12. [Quick Start](#12-quick-start)
13. [Dependencies](#13-dependencies)

---

## 1. Repository Layout

```
quad-koopman-datapipe/
├── README.md
│
├── ros_package/                   # ROS package: quad_data_collection
│   ├── package.xml
│   ├── CMakeLists.txt
│   └── scripts/
│       ├── data_collector.py      # Subscriber → CSV recorder
│       ├── trajectory_generator.py# Reference trajectory publisher
│       ├── pi_wrapper.py          # External integral action node
│       └── disturbance_publisher.py # Wind disturbance via Gazebo service
│
├── config/
│   ├── config.py                  # Single source of truth: dims, gains, vehicle params
│   └── quad_utils.py              # Motor mixer, quaternion helpers
│
├── collection/                    # Output directory (git-ignored)
│   ├── <traj>_v<ver>_<yaw>_seg_<N>.csv
│   └── .saved_seg_<N>             # save-complete sentinels
│
├── scripts/
│   └── run_collection.sh          # Orchestrator (Singularity-aware)
│
└── docs/
    └── new_dataset.md             # Detailed pipeline documentation
```

> **Note:** `collection/` (CSV outputs) is git-ignored. Add `collection/*.csv` to `.gitignore`.

---

## 2. Pipeline Overview

```
┌──────────────────────────────────────────────────────────────────┐
│  run_collection.sh  (Singularity container, optionally SLURM)    │
│                                                                  │
│   1. Start Xvfb + roscore                                        │
│   2. Start data_collector         (persistent, all segments)     │
│   3. Start pi_wrapper             (persistent, all segments)     │
│   4. Loop over 165 SCENARIOS:                                    │
│        restart Gazebo → reset PI integrator →                    │
│        launch disturbance_publisher →                            │
│        launch trajectory_generator →                             │
│        wait settle (12s) → start recording →                     │
│        record for (DURATION − 5)s →                              │
│        wait .saved_seg_N sentinel → kill children → next         │
└──────────────────────────────┬───────────────────────────────────┘
                               │
                    collection/*.csv
                               │
              ┌────────────────▼──────────────────┐
              │  inspect_sim_dataset.ipynb          │
              │  3D plots, RMSE table, drift check  │
              └────────────────┬──────────────────-┘
                               │
              ┌────────────────▼──────────────────┐
              │  csv_to_h5_sim.py                  │
              │  Stratified split by primitive type │
              │  → train_dataset.h5                │
              │  → val_dataset.h5                  │
              │  → test_dataset.h5                 │
              └────────────────┬──────────────────-┘
                               │
                        train_koopman.py
```

### Key design decisions

| Decision | Rationale |
|---|---|
| One gain set for all scenarios | Avoids per-scenario re-tuning; Lee gains tuned for 5 m/s peak |
| Persistent `data_collector` + `pi_wrapper` | Eliminates per-segment ROS node startup latency |
| Save-complete sentinel `.saved_seg_N` | Prevents next segment from racing the CSV write (~2–5s for 22k rows + Savitzky-Golay) |
| `DURATION − 5s` recording window | Avoids capturing trajectory_generator's Phase 6 hold-at-final-point, which inflates RMSE 5–10× on circles/harmonics |
| 100 Hz effective output via dedup filter | Timer runs at 400 Hz for responsiveness; rows emitted only on unique odometry timestamps |

---

## 3. ROS Nodes and Topics

### Node graph

```
                              /hummingbird/command/trajectory_unbiased
trajectory_generator ─────────────────────────────────────────────────► pi_wrapper
                                                                              │
                                                             /hummingbird/command/trajectory
                                                                              │
                                                                              ▼
Gazebo (RotorS) ──── /hummingbird/ground_truth/odometry ──────────► Lee Position Controller
       │         └── /hummingbird/imu                    │                    │
       │         └── /hummingbird/command/motor_speed ───┘           /hummingbird/command/motor_speed
       │                                                                       │
       │◄──────────────────────────── /gazebo/apply_body_wrench ──── disturbance_publisher
       │                                         ▲
       │                              /disturbance/state (50 Hz)
       │                                         │
       └───────────────────────────────────────► data_collector ──► collection/*.csv
```

### Topics table

| Topic | Type | Publisher | Subscriber(s) | Notes |
|---|---|---|---|---|
| `/hummingbird/ground_truth/odometry` | `nav_msgs/Odometry` | Gazebo/RotorS | `data_collector`, `pi_wrapper` | 100 Hz effective; position (world), velocity (body twist) |
| `/hummingbird/imu` | `sensor_msgs/Imu` | Gazebo/RotorS | `data_collector` | Linear acceleration (body, incl. gravity), angular velocity (body) |
| `/hummingbird/command/motor_speed` | `mav_msgs/Actuators` | Lee controller | `data_collector` | **NeuroBEM convention: magnitudes only, always positive.** Subscribing to `/motor_speed` (Gazebo plugin) was a v9 bug — it carries signed values and caused ~28% sign-flipped rows |
| `/hummingbird/command/trajectory_unbiased` | `trajectory_msgs/MultiDOFJointTrajectory` | `trajectory_generator` | `pi_wrapper` | Raw reference before integral correction |
| `/hummingbird/command/trajectory` | `trajectory_msgs/MultiDOFJointTrajectory` | `pi_wrapper` | Lee controller | Bias-corrected reference consumed by the flight controller |
| `/disturbance/state` | `std_msgs/Float32MultiArray` | `disturbance_publisher` | `data_collector` | `[mode_id, fx, fy, fz, _, _]`; recorded as `wind fx/fy/fz` columns |
| `/gazebo/apply_body_wrench` | Gazebo service | `disturbance_publisher` | Gazebo physics | Runtime per-segment wind injection at 50–100 Hz |
| `/collector/start` | `std_srvs/Trigger` | `run_collection.sh` | `data_collector` | Starts a recording session |
| `/collector/stop` | `std_srvs/Trigger` | `run_collection.sh` | `data_collector` | Stops recording and saves CSV |
| `/collector/status` | `std_srvs/Trigger` | `run_collection.sh` | `data_collector` | Returns `RECORDING`, `IDLE`, or `CRASHED:<reason>` |

### Node parameters (rosparams)

#### `data_collector`

| Param | Default | Description |
|---|---|---|
| `~output_dir` | `/data/collection` | CSV output directory |
| `~flight_name` | `flight` | Segment name prefix |
| `~record_hz` | `400.0` | Timer rate (Hz); effective output ~100 Hz via dedup |
| `~mav_name` | `hummingbird` | ROS namespace |
| `~sim_vbat` | `22.2` | Simulated battery voltage (V) — constant |
| `~sg_window` | `51` | Savitzky-Golay window length for `ang acc` and `dmot` |
| `~sg_polyorder` | `3` | Savitzky-Golay polynomial order |
| `~crash_z_min` | `0.30` | Crash threshold: z below this (m) triggers crash sentinel |
| `~crash_tilt_deg` | `60.0` | Crash threshold: tilt beyond this (deg) triggers crash sentinel |

#### `trajectory_generator`

| Param | Default | Description |
|---|---|---|
| `~traj_type` | — | `harmonic` \| `spline` \| `circle` \| `lissajous` \| `aggressive_sweep` \| `rose` \| `superellipse_track` |
| `~max_speed` | `2.0` | Target peak ground speed (m/s) |
| `~radius` | `3.0` | Circle/sweep radius (m) |
| `~duration` | — | Segment duration (s) |
| `~altitude` | `2.5` | Flight altitude (m) |
| `~yaw_mode` | `fixed` | `fixed` \| `sinusoidal` \| `rate_const` \| `velocity` |
| `~seed` | `42` | RNG seed (harmonic / spline) |
| `~n_modes` | `4` | Number of frequency modes (harmonic) |
| `~output_topic_suffix` | `command/trajectory` | Set to `command/trajectory_unbiased` when using pi_wrapper |

#### `pi_wrapper`

| Param | Default | Description |
|---|---|---|
| `~ki_xy` | `0.5` | XY integral gain |
| `~ki_z` | `1.0` | Z integral gain |
| `~i_limit_xy` | `1.0` | Anti-windup clamp, XY (m) |
| `~i_limit_z` | `0.5` | Anti-windup clamp, Z (m) |
| `~enable` | `true` | Disable to pass reference through unmodified |
| `/pi_wrapper/reset_seg_idx` | — | Shell sets this per-segment to trigger integrator reset |

#### `disturbance_publisher`

| Param | Default | Description |
|---|---|---|
| `~mode` | `none` | `none` \| `wind` |
| `~constant_force_n` | `0` | Steady wind magnitude (N) |
| `~constant_dir` | `"1,0,0"` | Steady wind direction (world frame, comma-separated) |
| `~gust_force_n` | `0` | Peak gust force (N) |
| `~gust_dir` | `"0,1,0"` | Gust direction (world frame) |
| `~gust_period_s` | `8` | Mean period between gusts (s) |
| `~gust_duration_s` | `2` | Per-gust duration (s) |
| `~seed` | `0` | RNG seed for gust schedule |

---

## 4. Dataset Schema

Each segment is saved as a CSV file with **32 columns** at ~100 Hz. The first 29 columns match the NeuroBEM format exactly, enabling any code trained on real NeuroBEM data to train directly on this simulated dataset.

### CSV column reference

| # | Column | Frame | Unit | Source | Notes |
|---|---|---|---|---|---|
| 1 | `t` | — | s | `odom.header.stamp` | Absolute sim time; used for dedup |
| 2–4 | `ang acc x/y/z` | **Body** | rad/s² | Derived | Savitzky-Golay derivative of `ang vel`, window=51, poly=3 |
| 5–7 | `ang vel x/y/z` | **Body** | rad/s | `/hummingbird/imu` | Raw IMU angular velocity |
| 8–11 | `quat x/y/z/w` | Body→World | — | `/hummingbird/ground_truth/odometry` | Hamilton convention, scalar-last |
| 12–14 | `acc x/y/z` | **Body** | m/s² | `/hummingbird/imu` | Linear acceleration incl. gravity (~9.8 m/s² on z at hover) |
| 15–17 | `vel x/y/z` | **Body** | m/s | `/hummingbird/ground_truth/odometry` twist | Body-frame linear velocity |
| 18–20 | `pos x/y/z` | **World** (ENU) | m | `/hummingbird/ground_truth/odometry` pose | |
| 21–24 | `mot 1/2/3/4` | — | rad/s | `/hummingbird/command/motor_speed` | Magnitudes only (NeuroBEM convention) |
| 25–28 | `dmot 1/2/3/4` | — | rad/s² | Derived | Savitzky-Golay derivative of `mot`, same window/poly |
| 29 | `vbat` | — | V | Constant | Simulated at 22.2 V (6S LiPo nominal) |
| 30–32 | `wind fx/fy/fz` | **World** | N | `/disturbance/state` | Applied wind force; zero for non-disturbed segments |

### Coordinate frame conventions

| Frame | Description |
|---|---|
| **World (ENU)** | East-North-Up, right-handed. Origin at spawn point. Used for position. |
| **Body (FLU)** | Forward-Left-Up, attached to drone. Used for angular velocity, linear acceleration, linear velocity. |
| **Quaternion** | Hamilton convention, scalar-last `[qx, qy, qz, qw]`. Encodes rotation from body to world. |

### Motor order (Hummingbird — PLUS config)

```
         Front (mot 1, CW)
              ↑
Left (mot 2, CCW) ── Body x → ── Right (mot 4, CCW)
              ↓
         Back (mot 3, CW)
```

Body frame: x forward, y left, z up.

### Filename convention

```
<traj_type>_v<max_speed>_<yaw_mode>_seg_<N>.csv
```

Examples:
- `harmonic_v2.5_ymvelocity_seg_21.csv` — harmonic trajectory, 2.5 m/s, velocity yaw mode, segment 21
- `aggressive_sweep_v3.5_ymfixed_seg_135.csv` — aggressive sweep, 3.5 m/s, fixed yaw, segment 135

---

## 5. Dataset Statistics (v10)

| Metric | Value |
|---|---|
| Total segments | **165** |
| Segment durations | 25–75 s (tailored per primitive) |
| Total flight time | **~180 minutes** |
| Total rows (post-dedup) | **~927,000** |
| Effective output rate | **~100 Hz** (timer 400 Hz, dedup on odom timestamp) |
| Trajectory primitives | **7** (circle, harmonic, spline, lissajous, aggressive_sweep, rose, superellipse_track) |
| Yaw modes | **4** (fixed, sinusoidal, rate_const, velocity) |
| Aperiodic content (spline) | **50%** (82/165 segments) |
| Wind-disturbed segments | **30%** (50/165 segments) |
| Wind configurations | 10 distinct (varied magnitude & direction) |
| Tracking median RMSE | **~9 cm** overall; best primitives 1.5–5 cm |
| Wall-clock collection time | **~5.5 hours** (SLURM limit: 7 h) |
| Completion rate | **100%** (165/165 OK in last 3 runs) |
| NeuroBEM column compatibility | **Yes** (first 29 columns identical) |

### Segment breakdown by primitive

| Primitive | Segments | Duration each | Speed range (m/s) |
|---|---|---|---|
| `circle` | 10 | 25 s | 2.0–4.5 |
| `harmonic` | 20 | 75 s | 1.5–4.0 |
| `spline` | 25 | 50 s | 1.5–4.5 |
| `lissajous` | 35 | 60 s | 1.5–4.5 |
| `aggressive_sweep` | 35 | 50 s | 1.0→3.5 to 1.0→5.0 |
| `rose` | 20 | 50 s | 1.5–3.5 |
| `superellipse_track` | 20 | 50 s | 1.5–4.0 |

### Yaw mode distribution

| Yaw mode | Count | ω_z range |
|---|---|---|
| `fixed` | ~60 | 0 rad/s |
| `sinusoidal` | ~35 | up to 0.2 rad/s peak |
| `rate_const` | ~35 | 0.2–0.5 rad/s constant |
| `velocity` | ~35 | up to 1.5 rad/s (slew-limited) |

---

## 6. Trajectory Primitives

All primitives enforce three safety rules by construction:
1. Yaw rate capped at `MAX_YAW_RATE = 1.5 rad/s`
2. Auto time-scaling: numerically probe peak speed and rescale to hit `max_speed` exactly
3. Ground clearance: z ≥ 0.5 m for every published reference point

| Primitive | Type | Key property |
|---|---|---|
| `circle` | Periodic | Closed horizontal circle. Canonical NeuroBEM-style reference. |
| `harmonic` | Periodic | `pos(t) = center + Σ_k Aₖ sin(ωₖ t + φₖ)`. Smooth to all derivatives. Bounded analytically. |
| `spline` | Aperiodic | Quintic B-spline through random 3D waypoints. C⁴ continuous, zero velocity/acceleration at boundaries (no jerk transients). |
| `lissajous` | Quasi-periodic | 3D Lissajous with rational frequency ratios. Rich box-coverage of (x, y, z). |
| `aggressive_sweep` | Ramp | Circle with linearly increasing tangential speed. Densely samples the (speed, curvature) cross-section. |
| `rose` | Periodic | Rose curve with optional vertical oscillation. |
| `superellipse_track` | Periodic | Superellipse (stadium shape) for track-like trajectories. |

---

## 7. Yaw Modes

| Mode | Formula | Max ω_z | Typical use |
|---|---|---|---|
| `fixed` | `yaw = const` | 0 | Splines (v→0 at waypoints makes `atan2` ill-defined) |
| `sinusoidal` | `yaw(t) = A·sin(2πft)`, A≈0.5–0.8 rad, f≈0.04–0.06 Hz | ~0.2 rad/s | Gentle oscillation on circles/harmonics |
| `rate_const` | `yaw(t) = ω·t`, ω ∈ [0.2, 0.5] rad/s | 0.5 rad/s | Steady yaw-translation coupling exercise |
| `velocity` | `yaw(t) = atan2(vy, vx)`, slew-rate-limited at 1.0–1.5 rad/s | 1.5 rad/s | Drone faces direction of travel; matches NeuroBEM dominant convention |

The `velocity` mode yaw is **pre-computed on a dense grid** at node startup, then interpolated at runtime, guaranteeing smooth quaternion references and preventing `atan2` jitter at low speed (hold threshold: 0.3 m/s).

---

## 8. Wind Disturbance Model

```
F_wind(t) = F_const · dir_const  +  F_gust(t) · dir_gust

F_gust(t) = gust_force · sin²(π · phase)   during gust windows
           = 0                              otherwise
```

Applied via Gazebo's `/gazebo/apply_body_wrench` service at 50–100 Hz. Force vectors recorded in `wind fx/fy/fz` columns (world frame, Newtons).

| Parameter | Range across dataset |
|---|---|
| Constant force `F_const` | 0.3–0.7 N |
| Gust peak force | 0.5–2.0 N |
| Gust mean period | 4–8 s |
| Gust duration | 1–2 s |
| Max total force | < 2.5 N (within Lee controller authority) |
| Wind configurations | 10 distinct direction/magnitude pairs |

No wind-induced crashes observed across any run. At evaluation time, the `wind fx/fy/fz` columns can be masked to test disturbance rejection.

---

## 9. PI Wrapper

The RotorS Lee controller is a pure PD law. Without integral action, constant disturbances (mass mismatch, centripetal undercommand) produce steady-state errors of ~5 cm altitude and ~3–7 cm radial drift on curved trajectories.

`pi_wrapper.py` adds external integral action:

```
ref_biased(t) = ref(t)  +  Ki · ∫₀ᵗ (ref − actual) dτ
```

This makes the Lee controller's PD law effectively PID on the original reference.

| Parameter | Value |
|---|---|
| `Ki_xy` | 0.5 |
| `Ki_z` | 1.0 |
| Anti-windup clamp XY | ±1.0 m |
| Anti-windup clamp Z | ±0.5 m |
| Integral time constant | ~16–24 s (Kp/Ki; slow, no transient interference) |

**Effect:**

| Metric | Without PI | With PI |
|---|---|---|
| Circle z_min (cmd 2.50 m) | 2.45–2.47 m | **2.48–2.49 m** |
| Circle median RMSE | 4–7 cm | **1.5–3.6 cm** |
| Aggressive sweep RMSE | 4–7 cm | **2.6–4.5 cm** |

The integrator is reset at the start of each segment via `/pi_wrapper/reset_seg_idx` rosparam.

---

## 10. Vehicle Parameters

Hummingbird quadrotor (RotorS/ETH Zürich). All values verified against `hummingbird.xacro` and `hummingbird.yaml` in RotorS commit `cd813b7`.

| Parameter | Symbol | Value | Unit |
|---|---|---|---|
| Mass | m | 0.716 | kg |
| Gravity | g | 9.8066 | m/s² |
| Inertia (diag) | I_xx, I_yy, I_zz | 0.007, 0.007, 0.012 | kg·m² |
| Hover thrust | T_hover | ~7.02 | N |
| Thrust coefficient | C_T | 8.54858×10⁻⁶ | N/(rad/s)² |
| Drag coefficient | C_Q | 1.36777×10⁻⁷ | N·m/(rad/s)² |
| Arm length | l | 0.17 | m |
| Max motor speed | ω_max | 838 | rad/s |
| Motor spool-up τ | τ_up | 0.0125 | s |
| Motor spool-down τ | τ_down | 0.025 | s |
| Max total thrust | T_max | ~24.0 | N (~3.4× hover) |
| Simulated battery | V_bat | 22.2 | V |

---

## 11. Koopman State Layout

Defined in `config/config.py` — single source of truth for all downstream training code.

```
x  =  [ p(3) | q_xyzw(4) | v(3) | ω(3) ]   dim = 13
         0:3    3:7         7:10   10:13

p  : position,         World frame (ENU)
q  : quaternion,       [qx, qy, qz, qw] scalar-last
v  : linear velocity,  World frame (rotated from body in preprocessing)
ω  : angular velocity, Body frame (IMU convention)

u  =  [ T, τ_x, τ_y, τ_z ]               dim = 4

Lifted state:
z  =  [ x(13) | kinematics(7) | observables(32) ]   dim = 52
```

Key training hyperparameters (all overridable via CLI):

| Param | Default | Description |
|---|---|---|
| `DT` | 0.01 s | Must match dataset rate (100 Hz sim, 0.0025 for NeuroBEM 400 Hz) |
| `OBS_DIM` | 32 | Learned nonlinear observables |
| `SEQ_LEN` | 50 | Trajectory window for all-pairs loss |
| `BATCH_SIZE` | 256 | |
| `LR` | 1e-3 | |
| `EPOCHS` | 100 | |
| `HIDDEN_DIM` | 128 | |
| `RHO_TARGET` | 0.99 | Spectral radius upper bound for stability penalty |

---

## 12. Quick Start

### Build the ROS package

```bash
cd ~/catkin_ws/src
git clone https://github.com/<you>/quad-koopman-datapipe.git
cd ..
catkin_make
source devel/setup.bash
```

### Run inside Singularity (cluster)

```bash
# Build container (once)
sudo singularity build quadrotor_sim.sif quadrotor_sim.def

# Submit SLURM job (16 CPUs, 32 GB, 7 h)
sbatch run_collection.slurm

# Or run directly inside container
singularity exec --bind /tmp,/dev/shm quadrotor_sim.sif bash run_collection.sh
```

### Manual single-segment test

```bash
roscore &
rosrun quad_data_collection data_collector.py _output_dir:=/tmp/test _flight_name:=test &
rosrun quad_data_collection pi_wrapper.py _mav_name:=hummingbird &
rosrun quad_data_collection trajectory_generator.py \
    _traj_type:=circle _max_speed:=2.0 _radius:=3.0 _duration:=30 \
    _altitude:=2.5 _yaw_mode:=fixed _yaw_const:=0.0 \
    _output_topic_suffix:=command/trajectory_unbiased
rosservice call /collector/start
sleep 25
rosservice call /collector/stop
```

### Convert to HDF5

```bash
python csv_to_h5_sim.py \
    --input_dir collection/ \
    --output_dir datasets/ \
    --train_frac 0.7 --val_frac 0.15 --test_frac 0.15
```

---

## 13. Dependencies

| Dependency | Version | Notes |
|---|---|---|
| ROS | Melodic / Noetic | Tested on Melodic inside Singularity |
| Gazebo | 9 / 11 | Headless via Xvfb |
| RotorS | — | ETH Zürich, commit cd813b7 |
| Python | 3.8+ | |
| numpy | — | |
| pandas | — | CSV I/O |
| scipy | — | Savitzky-Golay filter |
| h5py | — | HDF5 conversion |
| `mav_msgs` | — | `Actuators` motor speed message |
| `gazebo_msgs` | — | `ApplyBodyWrench` service |

---

## Citation / Reference

If you use this pipeline or dataset, please cite the NeuroBEM paper whose format this dataset mirrors:

> Bauersfeld, L., Kaufmann, E., Foehn, P., Sun, S., & Scaramuzza, D. (2021).
> *NeuroBEM: Hybrid Aerodynamic Quadrotor Model.*
> Robotics: Science and Systems (RSS).

---

## License

MIT
