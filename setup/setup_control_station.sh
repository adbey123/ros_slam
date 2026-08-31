#!/usr/bin/env bash
# Run this on laptop 2 (the control station -- this machine).
set -euo pipefail

echo "=== Installing control-station dependencies (needs sudo) ==="
sudo apt-get update
sudo apt-get install -y \
  ros-jazzy-slam-toolbox \
  ros-jazzy-rmw-cyclonedds-cpp \
  ros-jazzy-nav2-map-server \
  ros-jazzy-tf2-tools \
  python3-numpy python3-scipy python3-colcon-common-extensions

echo
echo "=== Building the workspace ==="
cd "$(dirname "$(readlink -f "$0")")/.."
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

echo
echo "Done. Next:"
echo "  1. ./setup/make_dds_config.sh <lidar-laptop-ip>"
echo "  2. source ./setup/ros_network_env.sh"
echo "  3. ros2 launch lidar_slam_bringup control_station.launch.py"
