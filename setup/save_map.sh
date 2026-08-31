#!/usr/bin/env bash
# Save the map slam_toolbox has built so far.
#
#   ./save_map.sh [name]        -> ~/lidar_slam_ws/maps/<name>.{pgm,yaml}
#
# Also writes a .posegraph so mapping can be resumed later via slam_toolbox's
# deserialize service.
set -euo pipefail

NAME="${1:-map_$(date +%Y%m%d_%H%M%S)}"
DIR="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)/maps"
mkdir -p "$DIR"

echo "Saving occupancy grid to $DIR/$NAME.{pgm,yaml}"
ros2 run nav2_map_server map_saver_cli -f "$DIR/$NAME" --ros-args -p save_map_timeout:=10.0

echo "Saving pose graph to $DIR/$NAME.posegraph"
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$DIR/$NAME'}"

echo "Done:"
ls -la "$DIR" | grep "$NAME"
