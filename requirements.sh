#!/usr/bin/env bash

set -e

echo "======================================"
echo " DriveSimROS2 Jazzy Dependencies"
echo " Target: Ubuntu 24.04 + ROS 2 Jazzy"
echo "======================================"

sudo apt update

echo
echo "Installing Python and build tools..."

sudo apt install -y \
    python3-pip \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential

echo
echo "Installing ROS 2 Jazzy packages..."

sudo apt install -y \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-vision-msgs \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-image

echo
echo "Installing Python requirements..."

python3 -m pip install --upgrade pip --break-system-packages
python3 -m pip install -r requirements.txt --break-system-packages

echo
echo "Dependency installation complete."