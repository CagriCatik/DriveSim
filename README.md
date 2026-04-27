<p align="center">
  <h1 align="center">DriveSim</h1>
</p>

<p align="center">
  <strong>Autonomous vehicle simulation for Ubuntu 24.04, ROS 2 Jazzy, and Gazebo Harmonic</strong>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/Ubuntu-24.04%20LTS-E95420?logo=ubuntu&logoColor=white" alt="Ubuntu 24.04 LTS">
  <img src="https://img.shields.io/badge/ROS%202-Jazzy%20Jalisco-22314E?logo=ros&logoColor=white" alt="ROS 2 Jazzy">
  <img src="https://img.shields.io/badge/Gazebo-Harmonic%208.x-F58113" alt="Gazebo Harmonic">
  <img src="https://img.shields.io/badge/build-colcon-blue" alt="colcon">
</p>

---

## Overview

**DriveSim** is a ROS 2-based autonomous vehicle simulation framework. The vehicle follows a circular road using a Stanley lateral controller, cubic-spline path interpolation, and a Bayesian occupancy filter for mapping. An interactive mode lets users place waypoints in RViz.

The project is derived from [AutoCarROS](https://github.com/winstxnhdw/AutoCarROS) and has been fully migrated to **Ubuntu 24.04 LTS**, **ROS 2 Jazzy Jalisco**, and **Gazebo Harmonic (gz-sim 8.x)**.

---

## Target Environment

| Component | Version |
|---|---|
| OS | Ubuntu 24.04 LTS |
| ROS 2 | Jazzy Jalisco |
| Gazebo | Harmonic (gz-sim 8.x) |
| Python | 3.12 |
| Build tool | colcon |

---

## Repository Structure

```text
DriveSim/
src/
  launches/              # ROS 2 launch files (default_launch.py, click_launch.py)
  autocar_description/   # URDF/Xacro robot description and RViz config
  autocar_gazebo/        # Gazebo Harmonic worlds, SDF model, meshes
  autocar_map/           # Bayesian occupancy filter (C++ node)
  autocar_msgs/          # Custom ROS 2 messages (Path2D, State2D, Twist2D)
  autocar_nav/           # Navigation stack (Python nodes + spline library)
```

---

## System Dependencies

### 1. ROS 2 Jazzy

```bash
# Follow https://docs.ros.org/en/jazzy/Installation.html
sudo apt install ros-jazzy-desktop
source /opt/ros/jazzy/setup.bash
```

### 2. Gazebo Harmonic + ROS bridge

```bash
sudo apt install ros-jazzy-ros-gz
```

This installs `ros_gz_sim`, `ros_gz_bridge`, and Gazebo Harmonic (gz-sim 8.x).

### 3. Build tools and Python deps

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep
sudo apt install python3-numpy ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-rviz2
pip3 install pandas  # required by globalplanner.py
```

---

## Build

```bash
cd /path/to/DriveSim   # e.g., ~/Desktop/DriveSim

# Install ROS package dependencies
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build

# Source
source install/setup.bash
```

### Verify packages are found

```bash
ros2 pkg list | grep autocar
# Expected output:
# autocar_description
# autocar_gazebo
# autocar_map
# autocar_msgs
# autocar_nav
```

---

## Usage

Always source both ROS 2 and the workspace before launching:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### Default simulation (preset waypoint loop)

Starts the vehicle on a circular road. It follows pre-recorded waypoints autonomously using the global planner and Stanley controller.

```bash
ros2 launch launches default_launch.py
```

### Interactive simulation (click-to-navigate)

Place at least 2 waypoints using **RViz's "2D Goal Pose"** tool. The vehicle will follow a cubic spline through them.

```bash
ros2 launch launches click_launch.py
```

### Optional: select a different world

```bash
ros2 launch launches default_launch.py world:=autocar_city.world
```

---

## Architecture

### Topic Flow

```
Gazebo Harmonic (gz-sim)
  /model/autocar/cmd_vel      <-- from ROS /autocar/cmd_vel (via bridge remapping)
  /model/autocar/odometry     --> ROS /autocar/odom (via bridge)
  /model/autocar/tf           --> ROS /tf (via bridge)
  /model/autocar/scan         --> ROS /scan (via bridge)

ROS 2 Navigation Stack
  /autocar/odom           --> localisation.py --> /autocar/state2D
  /autocar/state2D        --> global_planner  --> /autocar/goals
  /autocar/goals          --> local_planner   --> /autocar/path
  /autocar/path           --> tracker.py      --> /autocar/cmd_vel
  /autocar/odom + /scan   --> bof             --> /map
```

### Nodes

| Node | Package | Purpose |
|---|---|---|
| `localisation.py` | autocar_nav | Converts odometry to State2D |
| `globalplanner.py` | autocar_nav | Selects waypoints from CSV (2 Hz) |
| `localplanner.py` | autocar_nav | Cubic spline interpolation |
| `tracker.py` | autocar_nav | Stanley lateral controller |
| `clickplanner.py` | autocar_nav | Interactive waypoint planner |
| `bof` | autocar_map | Bayesian occupancy filter |

---

## Gazebo Classic to Gazebo Harmonic Migration

The original project used Gazebo Classic (Gazebo 11) with `gazebo_ros` plugins. This version uses Gazebo Harmonic (gz-sim 8.x) with native gz-sim plugins.

| Aspect | Gazebo Classic | Gazebo Harmonic |
|---|---|---|
| Drive plugin | `libgazebo_ros_ackermann_drive.so` | `gz-sim-ackermann-steering-system` |
| Laser sensor | `libgazebo_ros_ray_sensor.so` | `gpu_lidar` sensor type (native) |
| Launch | `gzserver` + `gzclient` executables | `gz_sim.launch.py` via `ros_gz_sim` |
| Bridge | `gazebo_ros` | `ros_gz_bridge` (parameter_bridge) |
| Model path | `GAZEBO_MODEL_PATH` | `GZ_SIM_RESOURCE_PATH` |
| SDF version | 1.5 | 1.8 |
| Odometry topic | `/autocar/odom` | `/model/autocar/odometry` (bridged) |
| cmd_vel topic | `/autocar/cmd_vel` | `/model/autocar/cmd_vel` (bridged) |

The bridge remaps Gazebo topics to the ROS names expected by nav nodes:
- `/model/autocar/cmd_vel` <-> `/autocar/cmd_vel`
- `/model/autocar/odometry` -> `/autocar/odom`

---

## Known Issues / Limitations

1. **`<road>` element in world**: The circular road is defined with `<road>` elements in SDF. Gazebo Harmonic supports this for visualization but may not render road textures identically to Gazebo Classic. A flat grey ground plane is included as fallback.

2. **Lidar bridge**: The lidar uses `gpu_lidar` sensor type. The `gz-sim-sensors-system` plugin requires a GPU-capable render engine (ogre2). On headless systems without GPU, sensor data may not publish. Use `--render-engine ogre` if ogre2 is unavailable.

3. **Pandas not in rosdep**: `pandas` is required by `globalplanner.py` but must be installed via `pip3 install pandas` as it is not in the ROS apt index.

4. **First waypoints.csv loop**: The waypoints form a circular arc. On first start, the global planner may publish the terminal waypoints briefly before tracking settles.

---

## Troubleshooting

### Gazebo window does not open

```bash
# Check gz-sim works independently
gz sim --verbose worlds/shapes.sdf
```

### Vehicle not spawning

```bash
# Verify GZ_SIM_RESOURCE_PATH includes the models directory
echo $GZ_SIM_RESOURCE_PATH
# The launch file sets this automatically. If running manually:
export GZ_SIM_RESOURCE_PATH=$(ros2 pkg prefix autocar_gazebo)/share/autocar_gazebo/models
```

### Nodes crash with "Missing ROS parameters"

The navigation nodes require the `navigation_params.yaml` to be loaded. When launched via `launch_launch.py`, parameters are passed automatically. If running nodes manually:

```bash
ros2 run autocar_nav localisation.py --ros-args \
  --params-file $(ros2 pkg prefix autocar_nav)/share/autocar_nav/config/navigation_params.yaml
```

### Build errors after modifying source

```bash
colcon build
source install/setup.bash
```

If symbols are stale:

```bash
rm -rf build install log
colcon build
source install/setup.bash
```

### ros2 pkg list does not show autocar packages

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 pkg list | grep autocar
```

---

## Credits

DriveSim is based on [AutoCarROS](https://github.com/winstxnhdw/AutoCarROS) by winstxnhdw.
Migrated to ROS 2 Jazzy + Gazebo Harmonic.
