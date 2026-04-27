<p align="center">
  <h1 align="center">DriveSim</h1>
</p>

<p align="center">
  <strong>Autonomous vehicle simulation for Ubuntu 24.04, ROS 2 Jazzy, and Gazebo Harmonic</strong>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/Ubuntu-24.04%20LTS-E95420?logo=ubuntu&logoColor=white" alt="Ubuntu 24.04 LTS">
  <img src="https://img.shields.io/badge/ROS%202-Jazzy%20Jalisco-22314E?logo=ros&logoColor=white" alt="ROS 2 Jazzy">
  <img src="https://img.shields.io/badge/Gazebo-Harmonic-F58113" alt="Gazebo Harmonic">
  <img src="https://img.shields.io/badge/build-colcon-blue" alt="colcon">
  <img src="https://img.shields.io/badge/license-TBD-lightgrey" alt="License">
</p>

---

## Overview

**DriveSim** is a ROS 2-based autonomous vehicle simulation framework for developing and testing a non-holonomic vehicle platform in a simulated environment.

The project is derived from the original [AutoCarROS](https://github.com/winstxnhdw/AutoCarROS) project and is being updated for a modern ROS 2 stack based on **Ubuntu 24.04**, **ROS 2 Jazzy Jalisco**, and **Gazebo Harmonic**.

It provides a reusable workspace for experimenting with vehicle control, navigation, mapping, custom ROS 2 messages, robot descriptions, Gazebo simulation assets, and launch-based system integration.

DriveSim can be used for:

- Autonomous vehicle simulation
- Vehicle control development
- Navigation and path planning
- Reactive behaviour testing
- Occupancy-grid mapping
- ROS 2 package development
- Gazebo-based robotics prototyping

---

## Target Environment

| Dependency | Version |
| --- | --- |
| Ubuntu | 24.04 LTS |
| ROS 2 | Jazzy Jalisco |
| Gazebo | Harmonic |
| Build tool | `colcon` |

> **Note**
>
> The original project targeted ROS 2 Foxy and Gazebo 11.
> This version targets ROS 2 Jazzy and Gazebo Harmonic. Some launch files, dependencies, and Gazebo integration packages may still require migration from Gazebo Classic to the modern Gazebo stack.

---

## Features

- ROS 2 Jazzy-based workspace
- Gazebo Harmonic simulation target
- Non-holonomic autonomous vehicle model
- Vehicle description and RViz configuration
- Gazebo worlds and simulation assets
- Custom ROS 2 message definitions
- Navigation and path-planning packages
- Mapping components
- Preset and interactive launch workflows

---

## Repository Structure

```text
DriveSim/
├── launches/              # Main ROS 2 launch files
├── autocar_description/   # Vehicle URDF and RViz configuration
├── autocar_gazebo/        # Gazebo worlds and vehicle simulation files
├── autocar_map/           # Mapping and occupancy-grid components
├── autocar_msgs/          # Custom ROS 2 message definitions
└── autocar_nav/           # Navigation, planning, and control components
````

---

## Installation

### 1. Install ROS 2 Jazzy

Follow the official ROS 2 Jazzy installation process for Ubuntu 24.04.

After installation, source ROS 2:

```bash
source /opt/ros/jazzy/setup.bash
```

To source ROS 2 automatically in future terminals:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

### 2. Install Gazebo and ROS-Gazebo packages

Install the Gazebo integration packages for ROS 2 Jazzy:

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz
```

---

### 3. Install build tools

```bash
sudo apt install python3-colcon-common-extensions
```

---

### 4. Create a ROS 2 workspace

```bash
mkdir -p ~/drivesim_ws/src
cd ~/drivesim_ws/src
```

---

### 5. Clone the repository

```bash
git clone https://github.com/YOUR_USERNAME/DriveSim.git
cd DriveSim
```

Replace `YOUR_USERNAME` with your GitHub username or organization name.

---

### 6. Install project dependencies

From the workspace root:

```bash
cd ~/drivesim_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

If the repository contains additional dependency scripts, review them before running them because older scripts may still target ROS 2 Foxy or Gazebo 11.

---

### 7. Build the workspace

```bash
cd ~/drivesim_ws
colcon build
```

---

### 8. Source the workspace

```bash
source install/setup.bash
```

To source the workspace automatically:

```bash
echo "source ~/drivesim_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Usage

Before launching the simulation, make sure both ROS 2 and the workspace are sourced:

```bash
source /opt/ros/jazzy/setup.bash
source ~/drivesim_ws/install/setup.bash
```

You can also rebuild before running:

```bash
cd ~/drivesim_ws
colcon build
source install/setup.bash
```

---

### Default Simulation

Runs the main autonomous vehicle simulation pipeline with preset waypoints.

```bash
ros2 launch launches default_launch.py
```

---

### Interactive Simulation

Runs an interactive simulation pipeline for testing, debugging, and manual waypoint selection.

```bash
ros2 launch launches click_launch.py
```

---

## Launch Files

| Launch file         | Description                                                                |
| ------------------- | -------------------------------------------------------------------------- |
| `default_launch.py` | Starts the complete simulation pipeline with preset waypoints.             |
| `click_launch.py`   | Starts the interactive pipeline for testing and manual waypoint selection. |

---

## Packages

| Package               | Description                                                    |
| --------------------- | -------------------------------------------------------------- |
| `launches`            | Main launch files for starting the simulation.                 |
| `autocar_description` | Vehicle URDF, model description, and RViz configuration files. |
| `autocar_gazebo`      | Gazebo worlds, SDF models, and simulation assets.              |
| `autocar_map`         | Mapping and occupancy-grid components.                         |
| `autocar_msgs`        | Custom ROS 2 message definitions shared across the project.    |
| `autocar_nav`         | Navigation, path planning, and control stack.                  |

---

## Migration Notes

This repository is being adapted from an older ROS 2 Foxy and Gazebo 11 setup to a newer ROS 2 Jazzy and Gazebo Harmonic setup.

Important migration areas include:

* Replacing Gazebo Classic-specific dependencies with `ros_gz` packages
* Updating launch files for the Jazzy environment
* Checking Python package compatibility with Ubuntu 24.04
* Updating deprecated ROS 2 APIs where necessary
* Verifying URDF, SDF, and Gazebo plugin compatibility
* Replacing old Gazebo Classic commands with modern Gazebo commands

Older Foxy/Gazebo 11 scripts such as:

```bash
ros-foxy-desktop-full-install.sh
```

should not be used on Ubuntu 24.04 unless they have been updated for Jazzy.

---

## Troubleshooting

### Build changes are not applied

If recent changes are not reflected after building, remove the generated build folders and rebuild:

```bash
cd ~/drivesim_ws
rm -rf build install log
colcon build
source install/setup.bash
```

---

### Package cannot be found

Make sure the workspace has been sourced:

```bash
source ~/drivesim_ws/install/setup.bash
```

Check whether ROS 2 can find the project packages:

```bash
ros2 pkg list | grep autocar
```

---

### ROS 2 command not found

Source the ROS 2 Jazzy setup file:

```bash
source /opt/ros/jazzy/setup.bash
```

To make this permanent:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

### Launch file does not start

Check that:

* Ubuntu 24.04 is being used.
* ROS 2 Jazzy is installed.
* Gazebo Harmonic or `ros-jazzy-ros-gz` is installed.
* The workspace was built successfully.
* The workspace has been sourced.
* Old Foxy or Gazebo 11 dependencies are not being used accidentally.

---

### Dependency errors during build

Try installing missing dependencies with `rosdep`:

```bash
cd ~/drivesim_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Then rebuild:

```bash
colcon build
source install/setup.bash
```

---

## Credits

DriveSim is based on the ROS 2 migration of [AutoCarROS](https://github.com/winstxnhdw/AutoCarROS).

The project has been renamed and updated to target a newer ROS 2 development environment.
