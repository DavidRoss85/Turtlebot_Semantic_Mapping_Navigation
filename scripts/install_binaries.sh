#!/bin/bash

set -e  # Exit on error

# ---------------------------------------------------------
# INSTALL SYSTEM-WIDE DEPENDENCIES (ROS 2 JAZZY + NAV2 + TB4)
# ---------------------------------------------------------

echo "[INFO] Updating APT..."
sudo apt update

echo "[INFO] Installing ROS 2 Jazzy core packages..."
sudo apt install -y \
    ros-jazzy-desktop \
    ros-jazzy-ros-base

# ---------------------------------------------------------
# NAVIGATION 2 PACKAGES
# ---------------------------------------------------------

echo "[INFO] Installing Nav2 binaries..."

sudo apt install -y \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-minimal-tb* \
    ros-jazzy-nav2-route

# ---------------------------------------------------------
# TURTLEBOT4 CORE PACKAGES
# ---------------------------------------------------------

echo "[INFO] Installing TurtleBot4 ROS packages..."

sudo apt install -y \
    ros-jazzy-turtlebot4-description \
    ros-jazzy-turtlebot4-msgs \
    ros-jazzy-turtlebot4-navigation \
    ros-jazzy-turtlebot4-node

# ---------------------------------------------------------
# GAZEBO (OPTIONAL – REQUIRED FOR SIMULATION)
# ---------------------------------------------------------

echo "[INFO] Installing Gazebo (ros-gz) dependencies..."

sudo apt install -y \
    curl \
    lsb-release \
    gnupg

echo "[INFO] Adding OSRF Gazebo package repository..."

sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
    --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
https://packages.osrfoundation.org/gazebo/ubuntu-stable \
$(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt update

echo "[INFO] Installing ROS ↔ Gazebo bridge..."
sudo apt install -y ros-jazzy-ros-gz

# ---------------------------------------------------------
# TURTLEBOT4 SIMULATION PACKAGES
# ---------------------------------------------------------

echo "[INFO] Installing TurtleBot4 simulation packages..."

sudo apt install -y \
    ros-jazzy-turtlebot4-simulator \
    ros-jazzy-irobot-create-nodes

# ---------------------------------------------------------
# TELEOP
# ---------------------------------------------------------

echo "[INFO] Installing teleop tools..."
sudo apt install -y ros-jazzy-teleop-twist-keyboard

# ---------------------------------------------------------
# ROSBRIDGE SERVER
# ---------------------------------------------------------

echo "[INFO] Installing rosbridge..."
sudo apt install -y ros-jazzy-rosbridge-server

echo "[INFO] System binary installation complete!"
echo "[INFO] Gazebo support installed (for simulation use)."
