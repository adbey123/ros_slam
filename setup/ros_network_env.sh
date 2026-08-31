# Source this in EVERY shell that runs a ROS2 node on EITHER laptop.
#
#   source ~/lidar_slam_ws/setup/ros_network_env.sh
#
# Both machines must agree on ROS_DOMAIN_ID and RMW_IMPLEMENTATION, or they will
# not discover each other no matter how good the network is.
#
# Override either before sourcing, to match a setup already in use:
#   LIDAR_SLAM_DOMAIN=10 LIDAR_SLAM_RMW=rmw_fastrtps_cpp source ros_network_env.sh

export ROS_DOMAIN_ID="${LIDAR_SLAM_DOMAIN:-42}"
export RMW_IMPLEMENTATION="${LIDAR_SLAM_RMW:-rmw_cyclonedds_cpp}"

# The unicast-peer profile applies to Cyclone only. See
# config/cyclonedds_wifi.xml.template for why the multicast default is a poor
# fit for WiFi. FastDDS works out of the box on a cooperative network, so it is
# a reasonable fallback if discovery is already working.
if [ "$RMW_IMPLEMENTATION" = "rmw_cyclonedds_cpp" ]; then
  if [ -f "$HOME/.ros/cyclonedds_wifi.xml" ]; then
    export CYCLONEDDS_URI="file://$HOME/.ros/cyclonedds_wifi.xml"
  else
    echo "[ros_network_env] WARNING: ~/.ros/cyclonedds_wifi.xml missing." >&2
    echo "[ros_network_env] Run: ~/lidar_slam_ws/setup/make_dds_config.sh <peer-ip>" >&2
  fi
else
  unset CYCLONEDDS_URI
fi

# Jazzy gates discovery by range; SUBNET lets the two laptops find each other.
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

source /opt/ros/jazzy/setup.bash
if [ -f "$HOME/lidar_slam_ws/install/setup.bash" ]; then
  source "$HOME/lidar_slam_ws/install/setup.bash"
fi

echo "[ros_network_env] domain=$ROS_DOMAIN_ID rmw=$RMW_IMPLEMENTATION"
