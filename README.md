# F1TENTH Simulator for Autonomous Vehicle Competitions
The F1TENTH simulator for ROS 2 is a simulation environment designed to test control, navigation, and planning algorithms on a 1/10 scale autonomous vehicle.

> ### 🏆 Sakhir GP Competition Project — Dual Reactive Controller Architecture
> Looking for the **Competition Reactive Controller documentation, code architecture, execution steps, and Oral Defense guide**?
> - 📘 **Full Technical Tutorial (Part B GitHub Documentation)**: [CONTROLLER_TUTORIAL.md](CONTROLLER_TUTORIAL.md)
> - 🛡️ **Version 1 (Endurance Champion — 10 Laps Guaranteed)**: [`src/controllers/controllers/gap_node.py`](src/controllers/controllers/gap_node.py) (Git tag: `v1.0-stable`, Best Lap: **38.88 s**, 0 Collisions, 0.534m Min Clearance)
> - ⚡ **Version 2 (Pure Real-Time LiDAR 9-Stage Adaptive Time-Attack)**: [`src/controllers/controllers/lidar_race_node.py`](src/controllers/controllers/lidar_race_node.py) & [`src/controllers/controllers/lidar_race_core.py`](src/controllers/controllers/lidar_race_core.py) (Best Live ROS 2 Lap: **38.891 s**, **+5.061 s** single-lap gain, **+37.603 s** total time saved across 10 laps, 0 Collisions, 0.537m Min Clearance, Active Steered Cooldown Braking)
> - 📊 **Telemetry & Time-Gain CSV Logs**: [`lap_times_sakhir.csv`](lap_times_sakhir.csv), [`lap_times_sakhir_v2_fast.csv`](lap_times_sakhir_v2_fast.csv), and [`lap_time_gains_sakhir.csv`](lap_time_gains_sakhir.csv)
>
> **Quick Competition Launch (Sakhir GP):**
> ```bash
> # Terminal 1: Launch Simulator + RViz2 (with 3D Lap HUD enabled)
> cd ~/F1Tenth-Repository && source /opt/ros/humble/setup.bash && source install/setup.bash
> ros2 launch f1tenth_gym_ros gym_bridge_launch.py
>
> # Terminal 2 (Option A): Launch Version 1 (Endurance Mode — 10 Consecutive Collision-Free Laps)
> cd ~/F1Tenth-Repository && source /opt/ros/humble/setup.bash && source install/setup.bash
> ros2 run controllers gap_node
>
> # Terminal 2 (Option B): Launch Version 2 (Qualifying Mode — Real-Time LiDAR Dynamic Racing)
> cd ~/F1Tenth-Repository && source /opt/ros/humble/setup.bash && source install/setup.bash
> ros2 run controllers lidar_race_node
> ```

**Features of the F1TENTH Simulator in ROS 2**
- Publishes and subscribes to ROS 2 topics, enabling interaction with virtual sensors and actuators.
- Simulates sensors such as LiDAR and odometry.
- Compatible with autonomous navigation algorithms.
- Supports 1 or 2 vehicles simultaneously.
- For a Spanish version of this file, click [here](README_es.md).

## 1. Complete Step-by-Step Installation & Setup Guide (From Zero to Running)

### 1.0 Supported Operating Systems & Software Versions Matrix

| Component | Recommended / Tested Version | Alternative Supported Versions | Notes |
| :--- | :--- | :--- | :--- |
| **Operating System** | **Ubuntu 22.04 LTS (Jammy Jellyfish)** (Native Linux or Windows 10/11 WSL2) | Ubuntu 20.04 LTS (Focal) / Ubuntu 24.04 LTS (Noble) | WSL2 on Windows 11 includes native WSLg GUI support for RViz2 |
| **ROS 2 Distribution** | **ROS 2 Humble Hawksbill** (LTS) | ROS 2 Foxy (Ubuntu 20.04) / ROS 2 Jazzy (Ubuntu 24.04) | Set `ROS_DISTRO=humble` (or `foxy` / `jazzy`) |
| **Python Version** | **Python 3.10.x** (`3.10.12` on Ubuntu 22.04) | Python 3.8 (Ubuntu 20.04) / Python 3.12 (Ubuntu 24.04) | Installed via `python3` & `python3-pip` |
| **Build System** | **`colcon`** (`python3-colcon-common-extensions`) + **`rosdep2`** | `ament_cmake` / `ament_python` | Builds ROS 2 packages in `src/` |
| **F1Tenth Physics Engine** | **`f1tenth_gym`** (`0.2.1` editable install) | — | RK4 integration + Pacejka tire model |
| **Core Python Libraries** | `gym==0.19.0`, `numpy==1.23.5` (or `<1.25`), `scipy>=1.7.0`, `numba>=0.55.0`, `Pillow>=9.0.0`, `pyyaml>=6.0`, `transforms3d>=0.3.1`, `shapely>=1.8.0` | `setuptools==58.2.0`, `pip<24.1` (required for `gym==0.19.0`) | Exact versions prevent `gym` build errors |

---

### 1.1 Step-by-Step ROS 2 Humble Installation on Ubuntu 22.04 (From Scratch)

If ROS 2 is not yet installed on your Ubuntu machine or WSL2 instance, run the following commands in order:

#### A. Configure UTF-8 System Locale
```bash
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

#### B. Enable the Ubuntu Universe Repository & Add Official ROS 2 GPG Key
```bash
sudo apt install -y software-properties-common curl gnupg lsb-release build-essential git
sudo add-apt-repository universe -y
sudo apt update

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

#### C. Install ROS 2 Desktop, Colcon Build Tools, and Required Simulation Packages
> **Note for Ubuntu 20.04 / 24.04**: Replace `humble` below with `foxy` (Ubuntu 20.04) or `jazzy` (Ubuntu 24.04).
```bash
sudo apt update && sudo apt upgrade -y

sudo apt install -y \
  ros-humble-desktop \
  ros-humble-ackermann-msgs \
  ros-humble-nav2-map-server \
  ros-humble-nav2-lifecycle-manager \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-teleop-twist-keyboard \
  ros-humble-tf-transformations \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pip \
  python3-vcstool
```

#### D. Initialize `rosdep`
```bash
sudo rosdep init || true
rosdep update
```

#### E. Add ROS 2 Environment to `~/.bashrc`
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

### 1.2 Install the `f1tenth_gym` Physics Simulator & Python Dependencies

The official `f1tenth_gym` requires `gym==0.19.0`, which relies on `setuptools<60` during installation. Run these exact commands so it installs cleanly without version conflicts:

```bash
# 1. Install compatible build tools and exact Python scientific packages
python3 -m pip install --upgrade "pip<24.1" "setuptools==58.2.0" "wheel<0.40.0"
python3 -m pip install \
  "gym==0.19.0" \
  "numpy==1.23.5" \
  "scipy>=1.8.0" \
  "numba>=0.56.0" \
  "Pillow>=9.0.0" \
  "PyYAML>=6.0" \
  "transforms3d>=0.4.1" \
  "shapely>=1.8.5" \
  "matplotlib>=3.5.0"

# 2. Clone and install f1tenth_gym in editable mode
cd $HOME
git clone https://github.com/f1tenth/f1tenth_gym.git
cd ~/f1tenth_gym
python3 -m pip install -e . --no-build-isolation
```
*(Note: If you are on **Ubuntu 24.04 (Python 3.12)**, append `--break-system-packages` to the `pip install` commands above).*

---

### 1.3 Clone `F1Tenth-Repository` & Install ROS Package Dependencies

```bash
cd $HOME
git clone https://github.com/Bishal-yeti/F1Tenth-Repository.git
cd ~/F1Tenth-Repository

# Install any remaining ROS 2 package dependencies declared in package.xml
source /opt/ros/humble/setup.bash
rosdep install -i --from-path src --rosdistro humble -y

# Remove python3-coverage if installed (prevents colcon/ament_python conflicts)
python3 -m pip uninstall -y coverage || true
sudo apt purge -y python3-coverage || true
```

https://github.com/user-attachments/assets/984a755c-6259-4271-9f88-51f68e425af5

https://github.com/user-attachments/assets/4c3ea5fb-4486-459f-beb7-bb7e8efa2a34

---

### 1.4 Configure the Map Path in `sim.yaml`

The simulator loads the track specified by `map_path` inside [`src/f1tenth_gym_ros/config/sim.yaml`](src/f1tenth_gym_ros/config/sim.yaml).

#### Automatic One-Line Configuration (Sets your current Linux username & Sakhir GP map):
```bash
sed -i "s|map_path:.*|map_path: '$HOME/F1Tenth-Repository/src/f1tenth_gym_ros/maps/Sakhir_map'|g" ~/F1Tenth-Repository/src/f1tenth_gym_ros/config/sim.yaml
```

#### Manual Configuration Details (`src/f1tenth_gym_ros/config/sim.yaml`):
- **For Sakhir GP (`Sakhir_map` — Default Competition Track)**:
  ```yaml
  map_path: '/home/your_user/F1Tenth-Repository/src/f1tenth_gym_ros/maps/Sakhir_map'
  map_img_ext: '.png'
  sx: 30.78
  sy: -22.72
  stheta: -2.635
  ```
- **For Spielberg (`Spielberg_map`) or Levine (`levine` / `levineB`)**:
  Replace `Sakhir_map` at the end of `map_path` with `Spielberg_map`, `levine`, or `levineB` and set `sx: 0.0, sy: 0.0, stheta: 0.0`.

https://github.com/user-attachments/assets/f66baff8-8d29-4306-a2e5-d5a50eb888f3

---

### 1.5 Build the ROS 2 Workspace (`colcon build`)

Whenever you clone the project or change `sim.yaml` / controller files, compile the workspace from the repository root:

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
rm -rf build/ install/ log/
colcon build --symlink-install
source install/setup.bash
```

---

### 1.6 Step-by-Step Execution Guide (Running the Simulator & Controllers)

You will use **two Ubuntu terminals** side-by-side: **Terminal 1** runs the physics bridge + RViz2 visualization, and **Terminal 2** runs the autonomous controller node.

#### Step 1 — Terminal 1: Launch the F1Tenth Simulator & RViz2
```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```
*Wait ~3–5 seconds for the RViz2 window to open and display the Sakhir GP circuit and the blue/black F1Tenth ego racecar at the starting line.*

https://github.com/user-attachments/assets/1791be99-73b6-4a22-acc2-6efa6fd40e5f

#### Step 2 — Terminal 2 (Option A): Run Version 2 — Pure Real-Time LiDAR 9-Stage Adaptive Racing Controller (Recommended)
```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run controllers lidar_race_node
```
- **What happens during execution**:
  - Drives **100% from live `/scan` LiDAR data** (zero pre-mapped waypoints).
  - Starts Lap 1 on the ultra-safe reference trim **`T0`** (`~43.9s`) and automatically promotes through **`T1 -> T8`** as verified LiDAR wall clearance stays clean, dropping lap times down to **`38.89s`** (`+5.06s` faster per lap, **`+37.60s` total time saved** across 10 laps).
  - Displays a live 3D floating HUD above the car in RViz2 (`/lap_hud_markers`), prints a live console ticker every 2 seconds, and outputs a full Unicode lap summary table at the end of each lap.
  - After completing **Lap 10**, automatically enters **Active Steered Cooldown Braking** (`speed = 0.0` while continuing to steer down the center of the straightaway until `velocity <= 0.05 m/s`), saves all telemetry to [`lap_times_sakhir_v2_fast.csv`](lap_times_sakhir_v2_fast.csv) and [`lap_time_gains_sakhir.csv`](lap_time_gains_sakhir.csv), and exits cleanly.

#### Step 2 — Terminal 2 (Option B): Run Version 1 — Endurance Fast-Centerline Controller
```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run controllers gap_node
```
- Runs the constant-trim endurance Follow-the-Gap controller (`38.88s` best lap, `0.534m` min clearance) and logs results to [`lap_times_sakhir.csv`](lap_times_sakhir.csv).

#### Step 2 — Terminal 2 (Option C): Run Headless 10-Lap Physics Benchmark (Without RViz2)
To verify 10 consecutive laps + post-Lap-10 cooldown braking directly against `f1tenth_gym` in ~15 seconds:
```bash
cd ~/F1Tenth-Repository
python3 benchmark_sakhir_lidar.py
```

#### Optional — Terminal 3: Monitor Live ROS 2 Lap Counter & Timer Topics
```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /lap_count          # Current completed lap number (Int32)
ros2 topic echo /lap_time           # Last completed lap time in seconds (Float32)
ros2 topic echo /best_lap_time      # Fastest lap time so far in seconds (Float32)
ros2 topic echo /race_telemetry     # JSON telemetry packet per lap (String)
```

## 2. Repository Structure

The repository provided was developed based on the various tasks that can be performed within the simulator. Additionally, it includes practical information provided by the competition organizers to help users learn key aspects of autonomous navigation.

<p align="center">
  <img src="img/repo.png" />
  <br />
  <em>Repository Contents</em>
</p>

#### 2.1. `Laboratories` Folder

This folder contains exercises provided by the competition organizers to help students learn how to use the simulator and the actual robot. There are a total of 7 exercises, covering the following topics:
- Lab 1: Introduction to ROS 2
- Lab 2: Automatic Emergency Braking
- Lab 3: Wall Follower (Control Algorithm)
- Lab 4: Follow the Gap (Control Algorithm)
- Lab 5: Pure Pursuit (Motion Planning)
- Lab 6: Rapidly-exploring Random Tree (RRT)
- Lab 7: Model Predictive Control (MPC)

#### 2.2. `Controllers` Package
As the name suggests, this folder is a ROS package containing several nodes designed to control the robot’s movement—that is, to send direction and speed messages to make the robot follow a specific behavior. It is important to note that these nodes are merely a set of basic functions and structures intended to serve as a guide for each student to write their own version of the code.

<p align="center">
  <img src="img/controllers.png" />
  <br />
  <em>Contents of the Controllers folder</em>
</p>

There are a total of 6 nodes, each corresponding to a lab exercise. The first lab exercise is an introduction to ROS, so it is not included in this folder.

- `ttc_node.py`: A node designed to contain the code necessary to ensure automatic emergency braking.
- `pid_node.py`: Node designed to contain a PID controller for tracking one of the walls on the map.
- `gap_node.py`: Node designed to contain the reactive `Follow the Gap` control and allow the robot to avoid obstacles.
- `purepursuit_node.py`: Node designed to contain the code necessary for the robot to follow a predefined path.
- `rrt_node.py`: Node designed to contain the code necessary to use the RRT algorithm and generate local trajectories that allow the robot to move and avoid obstacles.
- `mpc_node.py`: Node designed to contain the code necessary for the robot to use the `MPC` controller to move.

#### 2.3. `f1tenth_gym_ros` Package

This package contains everything needed for the `f1tenth_gym` simulator to work with ROS, allowing you to use the simulator with Rviz. Generally speaking, few changes will be made to this package, aside from the `maps` and `config` folders.

<p align="center">
  <img src="img/bridge.png" />
  <br />
  <em>Contents of the f1tenth_gym_ros folder</em>
</p>

**`maps` folder**

As the name suggests, this is where we’ll add all the maps we need for simulation. To add a map, we basically need two files:
- A `.png` file, which represents the image of our map.
- A `.yaml` file, which is a map configuration file in ROS 2 used for autonomous navigation.

Whenever a new map needs to be added, we need both files, which are generated using SLAM tools for simultaneous localization and mapping.

A basic structure of a `.yaml` file used for the simulator is as follows:

```bash
image: levineB.png
resolution: 0.050000
origin: [-25.0, -12.75, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196 que es este arhivo .yaml?
```

**Explanation of each parameter:**  
1. **`image: levineB.png`**  
   - This is the grayscale image representing the map of the environment.  
   - It is typically generated using tools such as **SLAM Toolbox**, **gmapping**, or **map_server**.  

2. **`resolution: 0.050000`**  
   - Defines the size of each pixel in **meters/pixel**.  
   - In this case, each pixel corresponds to **5 cm** in the real world.  

3. **`origin: [-25.0, -12.75, 0.000000]`**  
   - Defines the position of the map’s origin in the global coordinate system (X, Y, Theta).  
   - Here, the map origin is at (-25.0, -12.75) meters and **0 radians** of rotation.  

4. **`negate: 0`**  
   - Indicates whether the image color should be inverted.  
   - `0` means that **dark values** represent obstacles and **light values** are clear areas.  

5. **`occupied_thresh: 0.65`**  
   - Threshold limit for considering a pixel **occupied** (obstacle).  
   - If the pixel intensity is **greater than 65%**, it is considered an **obstacle**.  

6. **`free_thresh: 0.196`**  
   - Threshold limit for considering a pixel **free**.  
   - If the pixel intensity is **less than 19.6%**, it is considered a **navigable area**.  

**`config` folder**
This folder contains a single file named `sim.yaml`. This file contains the settings required for the simulator to function properly. Specifically, we will only modify three sections:

If you want to change the map being used, modify this line:
```bash
    map_path: '/home/israel/F1Tenth-Repository/src/f1tenth_gym_ros/maps/levine'
```

In this case, it uses the `levine.png` map. To ensure we use a different map, we need to place the PNG file in the maps folder and modify the end of the line mentioned above to include the name of our map.

https://github.com/user-attachments/assets/d1302653-d5e6-4022-9df9-a0113e9bdb12

The other section to be modified is:
```bash
    # opponent parameters
    num_agent: 1
```
Depending on whether we want 1 or 2 robots to appear in our simulation.

https://github.com/user-attachments/assets/a4a5d284-6bfa-42c0-ae58-5a4694da63ec

And finally:
```bash
    # ego starting pose on map
    sx: 0.0
    sy: 0.0
    stheta: 0.0

    # opp starting pose on map
    sx1: 9.5
    sy1: 8.5
    stheta1: 3.14
```
This defines the starting position of our robots on the map. If it is set to (0,0,0), the robot appears at the origin defined in the `.yaml` file. If we want it to appear at a different position, we assign values to `x` and `y` to change the position and to `tetha` to modify the orientation.

https://github.com/user-attachments/assets/6269e319-5558-458c-a9b0-5b7240a9e0cd

#### 2.4. `path_planning` Package

When developing autonomous navigation algorithms, it is necessary to include those that, based on a map, generate a global path while minimizing various parameters such as distance traveled, speed, and vehicle dynamics, among others.

This ROS package is designed to store all algorithms developed for the purpose of performing global route planning for our mobile robot.

<p align="center">
  <img src="img/path_planning.png" />
  <br />
  <em>Contents of the path_planning folder</em>
</p>

The package contains a file named `waypoint_recorder.py`, which serves as a template for a ROS 2 node for an odometry-based waypoint logger. Its purpose is to record the robot’s positions (x, y, orientation w) in a CSV file as it moves, with the goal of serving as the basis for developing a waypoint-based navigation system. It is one of the algorithms mentioned for the `Pure Pursuit` controller, which uses waypoints to follow a defined trajectory.

#### 2.5. `Waypoints` Folder

A folder designed to hold `.csv` files that represent the various waypoints needed to define an overall path for the mobile robot to follow.

**What are Waypoints?**  
**Waypoints** are reference points in space (generally expressed as (x, y, theta) coordinates) that indicate key positions a robot or vehicle must pass through during navigation.  

In the context of robotics and ROS 2, a waypoint is typically defined as:  
- **(x, y):** Position in the plane (or in 3D if aerial or underwater navigation is used).  
- **(theta) or (w):** The robot’s orientation at that point (in radians or quaternions).  

**Why are waypoints useful?**  
Waypoints are fundamental in multiple robotics and navigation applications:  

1. **Autonomous navigation:** They are used as waypoints to help a robot plan its path from a starting point to a destination.  

2. **Mapping and exploration:** In SLAM (Simultaneous Localization and Mapping), waypoints can mark explored locations or points of interest.  

3. **Trajectory tracking:** A drone or mobile robot can follow a series of predefined waypoints to move in a controlled manner.  

4. **Motion recording and analysis:** Waypoints can be stored to analyze a robot’s trajectory after a simulation or real-world test. 

## 3. Topics in the simulation
When we run the simulation with a single vehicle and use the `ros2 topic list` command, we get the following topics:

<p align="center">
  <img src="img/single_agent.png" />
  <br />
  <em>Topics with only one vehicle simulated</em>
</p>

The most important of these are:
- `/ego_racecar/odom`: This corresponds to the robot’s odometry, where we can obtain position and orientation data.
- `/scan`: A topic that sends messages from the LIDAR sensor, which provides information about the environment.
- `/map`: The simulation map, which contains `OccupancyGrip`-type messages.
- `/tf` and `/tf_static`: Publish the static and dynamic transformation trees present in the simulation.
- `/drive`: Where messages to move the robot are published, specifically `AckermannDriveStamped` messages.
- `/initialpose`: Topic that allows resetting the vehicle's position.

When we run the simulation with two vehicles and use the `ros2 topic list` command, we get the following topics:
<p align="center">
  <img src="img/two_agents.png" />
  <br />
  <em>Topics with 2 simulated vehicles</em>
</p>

In this case, we have the same topics as we did with a single vehicle in the simulation, and in addition, the topics for the opponent’s vehicle are added:
- `/opp_scan`: Messages from the opponent’s LIDAR.
- `/opp_racecar/odom`: The opponent’s odometry.
- `/opp_drive`: Where messages are published to move the opponent’s robot, such as `AckermannDriveStamped` messages.
- `/goal_pose`: A topic that allows you to reset the opponent’s vehicle position.

**Note:** When using two agents—that is, two vehicles in the simulation—we must simultaneously publish driving messages to the `/drive` and `/opp_drive` topics. If this is not done and commands are published to only one topic, neither of the two vehicles will move. The simulator detects that messages are being posted to both vehicle movement topics simultaneously, and only then does it allow movement for both; otherwise, it does not.

## 4. Keyboard Teleoperation
The `teleop_twist_keyboard` keyboard teleoperation node is also installed as part of the simulation dependencies. To enable keyboard teleoperation, set `kb_teleop` to True in `sim.yaml`. After launching the simulation, in another terminal, run:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
To move the vehicle in the simulator, use the following keys:
- `i`: to move forward.
- `u`: to move forward and to the left.
- `o`: to move forward and to the right.
- `k`: to stop all movement.
- `,`: to move backward.
- `m`: to move backward and to the left.
- `.`: to move backward and to the right

https://github.com/user-attachments/assets/1961d6bf-369a-4c58-9654-8d4c7cdefe19
