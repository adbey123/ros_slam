#!/usr/bin/env bash
# Run this on laptop 1 (the machine with the YDLIDAR attached).
#
# Copy this whole workspace over first, e.g.:
#   rsync -av --exclude build --exclude install --exclude log \
#         ~/lidar_slam_ws/ user@lidar-laptop:~/lidar_slam_ws/
set -euo pipefail

WS="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"

echo "=== Installing lidar-laptop dependencies (needs sudo) ==="
sudo apt-get update
sudo apt-get install -y \
  ros-jazzy-rmw-cyclonedds-cpp \
  ros-jazzy-tf2-tools \
  cmake build-essential git \
  python3-colcon-common-extensions

# --- YDLidar SDK ------------------------------------------------------------
# ydlidar_ros2_driver links against this; it is not packaged for Jazzy, so both
# the SDK and the driver are source builds.
SDK_DIR="$HOME/YDLidar-SDK"
if [[ ! -d "$SDK_DIR" ]]; then
  echo "=== Cloning YDLidar-SDK ==="
  git clone https://github.com/YDLIDAR/YDLidar-SDK.git "$SDK_DIR"
fi
echo "=== Building and installing YDLidar-SDK ==="
mkdir -p "$SDK_DIR/build"
cd "$SDK_DIR/build"
cmake ..
make -j"$(nproc)"
sudo make install
sudo ldconfig

# --- ROS2 driver ------------------------------------------------------------
DRIVER_DIR="$WS/src/ydlidar_ros2_driver"
if [[ ! -d "$DRIVER_DIR" ]]; then
  echo "=== Cloning ydlidar_ros2_driver ==="
  git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git "$DRIVER_DIR"
fi

# --- udev rule --------------------------------------------------------------
# Gives a stable /dev/ydlidar symlink and grants access without root, so you do
# not have to chase /dev/ttyUSB0 vs ttyUSB1 after every replug.
echo "=== Installing udev rule for /dev/ydlidar ==="
sudo tee /etc/udev/rules.d/99-ydlidar.rules >/dev/null <<'RULE'
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", GROUP="dialout", SYMLINK+="ydlidar"
KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE="0666", GROUP="dialout", SYMLINK+="ydlidar"
RULE
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -aG dialout "$USER" || true

echo
echo "=== Building the workspace ==="
cd "$WS"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

echo
echo "Done. Log out and back in once so the 'dialout' group takes effect, then:"
echo "  1. ./setup/make_dds_config.sh <control-station-ip>"
echo "  2. source ./setup/ros_network_env.sh"
echo "  3. ls -l /dev/ydlidar      # confirm the lidar enumerated"
echo "  4. ros2 launch lidar_slam_bringup lidar_laptop.launch.py lidar_model:=x4"
