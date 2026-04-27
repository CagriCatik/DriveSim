#!/usr/bin/env bash

set -e

echo "======================================"
echo " ROS 2 Jazzy Installation Script"
echo " Target: Ubuntu 24.04"
echo "======================================"

echo
echo "Current locale:"
locale

read -r -p "Do you want to update your locale to en_US.UTF-8? (y/n): " input

if [ "$input" = "y" ] || [ "$input" = "Y" ]; then
    sudo apt update
    sudo apt install -y locales

    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

    export LANG=en_US.UTF-8

    echo
    echo "Updated locale:"
    locale
else
    echo "Proceeding without updating locale."
fi

echo
echo "Installing required system packages..."

sudo apt update
sudo apt install -y \
    software-properties-common \
    curl \
    gnupg \
    lsb-release

echo
echo "Enabling Ubuntu Universe repository..."

sudo add-apt-repository -y universe

echo
echo "Adding ROS 2 apt repository..."

sudo apt update
sudo apt install -y curl gnupg lsb-release

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo
echo "Installing ROS 2 Jazzy..."

sudo apt update
sudo apt install -y ros-jazzy-desktop

echo
echo "Installing Gazebo / ROS-Gazebo integration packages..."

sudo apt install -y ros-jazzy-ros-gz

echo
echo "Installing colcon and development tools..."

sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential

echo
echo "Initializing rosdep..."

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi

rosdep update

echo
echo "Running project dependency script if available..."

if [ -f requirements.sh ]; then
    sh requirements.sh
else
    echo "requirements.sh not found. Skipping project-specific dependency script."
fi

echo
echo "Updating .bashrc..."

if ! grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
fi

if ! grep -q "source /usr/share/colcon_cd/function/colcon_cd.sh" ~/.bashrc; then
    echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
fi

if ! grep -q "export _colcon_cd_root=~/drivesim_ws" ~/.bashrc; then
    echo "export _colcon_cd_root=~/drivesim_ws" >> ~/.bashrc
fi

echo
echo "Sourcing ROS 2 Jazzy for current terminal..."

source /opt/ros/jazzy/setup.bash

echo
echo "Installation complete."
echo
echo "Run this command now or open a new terminal:"
echo "source ~/.bashrc"
echo
echo "Check ROS 2 with:"
echo "ros2 --version"