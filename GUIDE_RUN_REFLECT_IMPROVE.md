# 🏎️ F1TENTH Autonomous Racing: Master Guide to Run, Reflect, and Improve

Welcome to the **Zero-Search, Step-by-Step Engineering Guide** for the F1TENTH Autonomous Racing Suite. Whether you are running this on native Ubuntu Linux, a fresh PC, or Windows 10/11 using WSL2, this guide walks you through **running**, **reflecting on telemetry**, and **tuning/improving** your racing AI without needing to search or guess.

---

## 📑 Table of Contents
1. [⚡ Part 1: Step-by-Step Running Guide (Zero to Racing)](#1-step-by-step-running-guide-zero-to-racing)
   - [A. Fresh PC / Windows WSL2 1-Command Setup](#a-fresh-pc--windows-wsl2-1-command-setup)
   - [B. Method 1: The 1-Command All-in-One Race (Recommended!)](#b-method-1-the-1-command-all-in-one-race-recommended)
   - [C. Method 2: The Two-Terminal Standard ROS 2 Workflow](#c-method-2-the-two-terminal-standard-ros-2-workflow)
   - [D. Controller Selection Guide](#d-controller-selection-guide)
   - [E. What You See on Screen During a Race](#e-what-you-see-on-screen-during-a-race)
2. [📊 Part 2: How to Reflect on Telemetry & Race Data](#2-how-to-reflect-on-telemetry--race-data)
   - [A. Live Graphical Dashboard Breakdown](#a-live-graphical-dashboard-breakdown)
   - [B. CSV Logs & Metric Reference Table](#b-csv-logs--metric-reference-table)
   - [C. Telemetry Reflection Checklist (Grading & Oral Defense)](#c-telemetry-reflection-checklist-grading--oral-defense)
3. [🛠️ Part 3: Step-by-Step Controller Improvement & Tuning Guide](#3-step-by-step-controller-improvement--tuning-guide)
   - [A. Master Symptom-Diagnosis-Fix Decision Matrix](#a-master-symptom-diagnosis-fix-decision-matrix)
   - [B. Parameter Code Map (Exact Files & Line Numbers)](#b-parameter-code-map-exact-files--line-numbers)
   - [C. How to Create Custom Pace Trims](#c-how-to-create-custom-pace-trims)
   - [D. Headless Physics Benchmarking (10x Faster Tuning)](#d-headless-physics-benchmarking-10x-faster-tuning)
   - [E. Racing on All 12 Bundled F1Tenth Circuits](#e-racing-on-all-12-bundled-f1tenth-circuits)

---

## 1. Step-by-Step Running Guide (Zero to Racing)

### A. Fresh PC / Windows WSL2 1-Command Setup

If you are on a fresh PC or on Windows (using Ubuntu 22.04 in WSL2):

```bash
# 1. Open your Ubuntu / WSL2 terminal
# 2. Clone the repository
git clone https://github.com/Bishal-yeti/F1Tenth-Repository.git
cd F1Tenth-Repository

# 3. Run the automated installer (installs ROS 2 Humble, f1tenth_gym, Python libraries & builds packages)
./setup_f1tenth.sh
```

> **Windows 10/11 WSL2 Users**: If using WSL2, Windows 11 includes **WSLg** built-in. Graphical windows (RViz2 and the Tkinter Speedometer GUI) will appear on your Windows desktop automatically!

---

### B. Method 1: The 1-Command All-in-One Race (Recommended!)

You do **NOT** need multiple terminals or complex commands. You can launch the Simulator, RViz2 3D HUD, and the Autonomous AI Controller together in **one single command**:

```bash
./run_race.sh race
```
*(Or simply run `./run_race.sh` and press **Enter** — Option `[1]` is the default!)*

```text
==============================================================================
  🏎️  STARTING ALL-IN-ONE AUTONOMOUS RACE (SIMULATOR + RVIZ2 + AI CONTROLLER)
==============================================================================
  Track : Sakhir_map
  AI    : Stable 10-Stage Adaptive LiDAR (V2)
  Press Ctrl+C at any time to stop everything.
==============================================================================
```

**What happens automatically:**
1. ROS 2 starts the 100 Hz RK4 Gym physics simulator.
2. RViz2 opens with the Sakhir track and 3D HUD pre-loaded.
3. The Live Speedometer & Lap-Time GUI window pops up.
4. After a 2-second initialization delay, the AI controller connects and the car immediately launches off the starting line!
5. Pressing `Ctrl+C` cleanly shuts down all processes.

---

### C. Method 2: The Two-Terminal Standard ROS 2 Workflow

If you prefer separate terminals to monitor simulator and controller logs independently:

#### Terminal 1 — Start the Simulator & RViz2:
```bash
./run_race.sh sim
```
*You will see the following on-screen reminder:*
```text
╔══════════════════════════════════════════════════════════════════════════════╗
║  🏎️  STEP 1/2: F1TENTH SIMULATOR & RVIZ2 STARTING (TERMINAL 1)                 ║
╠══════════════════════════════════════════════════════════════════════════════╣
║  ⚠️  IMPORTANT NOTICE:                                                       ║
║      The racecar will NOT move yet! It is waiting at the start line for an   ║
║      autonomous controller to publish steering and speed commands.           ║
║                                                                              ║
║  👉 STEP 2: OPEN A SECOND TERMINAL WINDOW (OR SPLIT PANE) AND RUN:           ║
║         cd F1Tenth-Repository                                                ║
║         ./run_race.sh lidar      # Stable 10-Stage Adaptive AI (Recommended) ║
║      OR:                                                                     ║
║         ./run_race.sh qualifier  # Fast-Start Qualifier AI (Q-T6->Q-T8)      ║
║         ./run_race.sh gap        # Version 1 Endurance AI                    ║
╚══════════════════════════════════════════════════════════════════════════════╝
```

#### Terminal 2 — Start Your AI Controller:
```bash
# Recommended: Stable 10-Stage Adaptive AI (starts safe on T0, evolves to T8/T9)
./run_race.sh lidar

# OR: Fast-Start Qualifier AI (starts directly at Q-T6 for fast 4-lap sprint)
./run_race.sh qualifier

# OR: Version 1 Endurance AI (constant-trim champion)
./run_race.sh gap
```

---

### D. Controller Selection Guide

| Controller | Command | Strategy | Fastest Lap | Min Clearance | Best Use Case |
| :--- | :--- | :--- | :---: | :---: | :--- |
| **V2: Stable Adaptive** (`lidar_race_node`) | `./run_race.sh lidar` | Starts on safe anchor `T0`, automatically promotes up to `T8`/`T9` + Straightaway DRS + Active Cooldown Stop | **`38.490 s`** | `0.495 – 0.665 m` | **Course Grading & 10 Consecutive Collision-Free Laps** |
| **V3: Fast-Start Qualifier** (`qualifier_node`) | `./run_race.sh qualifier` | Skips `T0–T5` warmup, launches directly in high-attack `Q-T6` $\rightarrow$ `Q-T7` $\rightarrow$ `Q-T8` with instant safety fallback | **`38.410 s`** | `0.494 – 0.593 m` | **Single Fastest Lap Competition** |
| **V1: Endurance Champion** (`gap_node`) | `./run_race.sh gap` | Constant-trim, rock-solid baseline (`v1.0-stable`) | **`38.880 s`** | `0.534 m` | **Endurance Consistency Benchmark** |

---

### E. What You See on Screen During a Race

```text
┌──────────────────────────────────────┐  ┌──────────────────────────────────────┐
│             RViz 2 Window            │  │      Live Telemetry GUI Window       │
│                                      │  │                                      │
│  [3D Floating HUD above Racecar]     │  │  SPEED: 16.17 m/s  (58.2 km/h)      │
│  Trim: T8 | Lap: 6/10 | Time: 38.49s │  │  [=============>                ]   │
│  Dist: 438.8m | Speed: 16.0 m/s      │  │                                      │
│                                      │  │  CURRENT LAP: 6/10  TIME: 38.49 s    │
│  [3D LiDAR Target Sphere]            │  │  FASTEST LAP: 38.49 s                │
│  Green sphere marking chosen ray     │  │  TOTAL GAIN : +18.80 s vs Lap 1      │
│                                      │  │                                      │
│  [3D Orange Trajectory Trail]        │  │  LAP RECORDS TABLE:                  │
│  Real-time driven racing line        │  │  Lap 1 [T0]: 43.76s (Gain: +0.00s)   │
│                                      │  │  Lap 2 [T1]: 42.28s (Gain: +1.48s)   │
│                                      │  │  Lap 6 [T8]: 38.49s (Gain: +5.27s) ★ │
└──────────────────────────────────────┘  └──────────────────────────────────────┘
```

---

## 2. How to Reflect on Telemetry & Race Data

Reflection is critical in robotics and autonomous systems. After each race, you have real-time and post-run logs to examine:

### A. Live Graphical Dashboard Breakdown

When running `lidar_race_node` or `qualifier_node`, the Tkinter dashboard displays:
- **Digital Speedometer**: Live speed in both metric $\text{m/s}$ and $\text{km/h}$.
- **Speed Gauge Bar**: Dynamic colored bar indicating throttle vs maximum trim speed.
- **Lap Status & Stopwatch**: Real-time simulation stopwatch updated at 10 Hz.
- **Pace Trim Indicator**: Live trim (`T0` through `T9` in V2; `Q-T6` through `Q-T8` in V3).
- **Minimum Clearance Gauge**: Live distance to the nearest barrier (alert turns yellow $< 0.50\text{ m}$, red $< 0.30\text{ m}$).
- **Lap-by-Lap Records Table**: Real-time summary showing Lap Time, Gain vs Lap 1, Peak Velocity, and Min Clearance.

---

### B. CSV Logs & Metric Reference Table

Every run automatically writes full telemetry to CSV files in the repository root:
- [`lap_times_sakhir_v2_fast.csv`](lap_times_sakhir_v2_fast.csv): Comprehensive 12-column telemetry log.
- [`lap_time_gains_sakhir.csv`](lap_time_gains_sakhir.csv): Time-gains and trim parameters log.

#### What Each Column Means:
| Column Header | Unit | Meaning | Ideal Value |
| :--- | :---: | :--- | :---: |
| `lap_number` | — | Current completed lap index (`1..10`) | `10` |
| `pace_trim` | — | Active pace trim (`T0..T9`) | Progresses `T0 -> T8` |
| `lap_time_sec` | seconds | Total time taken to complete the lap | $< 39.0\text{ s}$ |
| `time_gained_vs_lap1_sec` | seconds | Time shaved off compared to the sighting lap (Lap 1) | $> +4.5\text{ s}$ |
| `delta_vs_prev_lap_sec` | seconds | Time gained or lost compared to the immediately preceding lap | Positive ($> 0$) |
| `cumulative_time_saved_sec`| seconds | Total accumulated time saved across the session | $> +30.0\text{ s}$ |
| `min_wall_clearance_m` | meters | Closest proximity of any LiDAR ray to a track barrier | `0.48 – 0.65 m` ($> 0.31\text{ m}$ car width) |
| `max_speed_mps` | $\text{m/s}$ | Peak velocity achieved on the main straight | $> 16.0\text{ m/s}$ ($57.6\text{ km/h}$) |
| `lap_distance_m` | meters | Integrated distance traveled along the vehicle path | $\sim 438\text{ m}$ (efficient racing line) |

---

### C. Telemetry Reflection Checklist (Grading & Oral Defense)

Use this checklist to reflect on your run:
1. **Safety**: Did the car complete all 10 laps with **0 collisions**?
   - Verified if `min_wall_clearance_m > 0.31 m` (physical car width). A clearance of `0.48 – 0.65 m` is optimal.
2. **Progressive Gain**: Did lap times decrease from Lap 1 to Lap 6?
   - Sighting lap: `~43.7 s` (`T0`).
   - Flying lap: `~38.4 s` (`T8`).
   - Time gain: `> 5.2 s` single-lap gain; `> 35.0 s` cumulative time saved.
3. **Controlled Cooldown**: Did the car perform an **Active Steered Cooldown Stop** after Lap 10?
   - Check console log: car should decelerate to `0.05 m/s` while staying in the center of the track (`min_clearance > 0.90 m`).

---

## 3. Step-by-Step Controller Improvement & Tuning Guide

Want to make the car faster, smoother, or adapt it to a new track? Use the decision matrix below:

### A. Master Symptom-Diagnosis-Fix Decision Matrix

| Symptom / Behavior | Root Cause | Exact Parameter to Adjust | Where to Find It |
| :--- | :--- | :--- | :--- |
| **Car hits outside wall on corner entry** *(Overshooting / Understeer)* | Braking initiates too late, or lateral grip limit is set too high | 1. Increase `brake_entry_gain` (e.g. `0.15` $\rightarrow$ `0.18`)<br>2. Increase `safety_margin` (e.g. `1.60` $\rightarrow$ `1.80`)<br>3. Lower `min_speed` (e.g. `4.2` $\rightarrow$ `3.8`) | [`lidar_race_core.py`](src/controllers/controllers/lidar_race_core.py) (`DEFAULTS_V2`) |
| **Car clips inside barrier in chicanes** *(Apex clipping)* | Disparity extender safety width is narrower than actual chassis envelope | 1. Increase `car_width_safety` (e.g. `0.35` $\rightarrow$ `0.38`)<br>2. Ensure `inside_width_scale = 1.0` (symmetrical) | [`lidar_race_core.py`](src/controllers/controllers/lidar_race_core.py) (`DEFAULTS_V2`) |
| **Car oscillates / snakes on straightaways** *(High-speed hunting)* | Micro-steering corrections throttle velocity or derivative gain is undamped | 1. Increase `deadband_rad` (e.g. `0.038` $\rightarrow$ `0.045 rad` / $2.5^\circ$)<br>2. Increase `steer_smoothing` (e.g. `0.10` $\rightarrow$ `0.15`) | [`lidar_race_core.py`](src/controllers/controllers/lidar_race_core.py) (`DEFAULTS_V2`) |
| **Car turns backward or gets confused in 180° hairpins** | LiDAR FOV is seeing straightaways *behind* the front axle | 1. Ensure `fov_angle_deg = 140.0` ($\pm 70^\circ$, never $270^\circ$)<br>2. Keep `use_weighted_center = 0.08` | [`lidar_race_core.py`](src/controllers/controllers/lidar_race_core.py) (`DEFAULTS_V2`) |
| **Car does not reach top speed on straightaways** *(DRS not firing)* | DRS activation thresholds are too strict for track straight length | 1. Lower `self.prev_front` threshold (e.g. `16.0` $\rightarrow$ `14.0`)<br>2. Lower `raw_front_p40` threshold (e.g. `18.5` $\rightarrow$ `17.0`) | [`lidar_race_core.py`](src/controllers/controllers/lidar_race_core.py) (`step()`) |
| **Car triggers mid-lap downshift too frequently** | Safety threshold `0.48m` is triggered by normal close curb passes | 1. Adjust downshift threshold in `check_mid_lap_safety` (`0.48` $\rightarrow$ `0.45`)<br>2. Check for sensor noise | [`lidar_race_core.py`](src/controllers/controllers/lidar_race_core.py) (`check_mid_lap_safety`) |

---

### B. Parameter Code Map (Exact Files & Line Numbers)

| Parameter | Default | File Path | Role |
| :--- | :---: | :--- | :--- |
| `fov_angle_deg` | `140.0` | [`src/controllers/controllers/lidar_race_core.py`](src/controllers/controllers/lidar_race_core.py) | Forward perception cone. Eliminates rear-straight false gaps in $180^\circ$ hairpins. |
| `car_width_safety` | `0.35` | [`src/controllers/controllers/lidar_race_core.py`](src/controllers/controllers/lidar_race_core.py) | Virtual chassis width for disparity extension ($> 0.31\text{ m}$ physical car). |
| `bubble_radius` | `0.52` | [`src/controllers/controllers/lidar_race_core.py`](src/controllers/controllers/lidar_race_core.py) | Dynamic zeroing bubble around closest obstacle point. |
| `brake_entry_gain` | `0.15` | [`src/controllers/controllers/lidar_race_core.py`](src/controllers/controllers/lidar_race_core.py) | Closing-rate dynamic trail-braking multiplier: $d_{\text{dyn}} = d_0 + k_{\text{entry}}\,\Delta d \cdot v$. |
| `deadband_rad` | `0.038` | [`src/controllers/controllers/lidar_race_core.py`](src/controllers/controllers/lidar_race_core.py) | Straightaway steering deadband ($2.18^\circ$) on lateral acceleration calculation. |
| `steer_gain` | `0.76` | [`src/controllers/controllers/lidar_race_core.py`](src/controllers/controllers/lidar_race_core.py) | Proportional steering gain scheduled with vehicle speed. |
| `steer_deriv_gain` | `0.15` | [`src/controllers/controllers/lidar_race_core.py`](src/controllers/controllers/lidar_race_core.py) | Derivative steering damping gain on deduplicated LiDAR scans. |
| `gate_radius` | `5.0` | [`src/controllers/controllers/lidar_race_node.py`](src/controllers/controllers/lidar_race_node.py) | Radius of start/finish detection gate (covers full track corridor width). |

---

### C. How to Create Custom Pace Trims

In [`src/controllers/controllers/lidar_race_core.py`](src/controllers/controllers/lidar_race_core.py), locate `PACE_TRIMS`:

```python
PACE_TRIMS = [
    dict(min_speed=4.20, a_lat_max=9.00,  a_brake=7.20, safety_margin=1.60, max_speed=16.20), # T0 (Safe Anchor)
    dict(min_speed=4.50, a_lat_max=9.55,  a_brake=7.40, safety_margin=1.50, max_speed=16.70), # T1
    dict(min_speed=4.80, a_lat_max=10.15, a_brake=7.55, safety_margin=1.42, max_speed=17.10), # T2
    dict(min_speed=5.10, a_lat_max=10.75, a_brake=7.68, safety_margin=1.35, max_speed=17.50), # T3
    dict(min_speed=5.35, a_lat_max=11.25, a_brake=7.76, safety_margin=1.28, max_speed=17.75), # T4
    dict(min_speed=5.55, a_lat_max=11.65, a_brake=7.85, safety_margin=1.22, max_speed=17.95), # T5
    dict(min_speed=5.75, a_lat_max=12.00, a_brake=7.95, safety_margin=1.19, max_speed=18.15), # T6
    dict(min_speed=5.95, a_lat_max=12.35, a_brake=8.02, safety_margin=1.17, max_speed=18.35), # T7
    dict(min_speed=6.12, a_lat_max=12.65, a_brake=8.08, safety_margin=1.15, max_speed=18.60), # T8
    dict(min_speed=6.25, a_lat_max=12.90, a_brake=8.15, safety_margin=1.14, max_speed=18.85), # T9
]
```

**Tuning Rules:**
1. **`min_speed`**: Controls minimum speed through the tightest $180^\circ$ hairpins.
2. **`max_speed`**: Controls straightaway speed limit.
3. **`a_lat_max`**: Higher values allow higher entry speeds into medium-radius sweepers.
4. **`safety_margin`**: Forward obstacle runway distance for braking.

---

### D. Headless Physics Benchmarking (10x Faster Tuning)

Instead of launching RViz2 every time you tweak a parameter, run the headless physics benchmark:

```bash
./run_race.sh bench
# OR directly:
python3 benchmark_sakhir_lidar.py
```

- Simulates 10 full laps in **under 20 seconds** (10x faster than real-time).
- Runs 100% accurate 100 Hz RK4 physics and Pacejka tire dynamics.
- Immediately outputs the completed lap times, gains, minimum clearances, and collisions table!

---

### E. Racing on All 12 Bundled F1Tenth Circuits

You can switch to any of the 12 bundled circuits with one command:

```bash
./switch_map.sh
```
Or specify the track directly:
```bash
./switch_map.sh Zandvoort_map      # Dutch Grand Prix Circuit
./switch_map.sh Catalunya_map      # Spanish Grand Prix Circuit
./switch_map.sh Sepang_map         # Malaysian Grand Prix Circuit
./switch_map.sh BrandsHatch_map    # British Circuit
./switch_map.sh Sakhir_map         # Return to Bahrain Sakhir GP
```

Every circuit comes with a **pre-configured drivable start pose `(sx, sy, stheta)`** so the car never spawns inside a wall!
