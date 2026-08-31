#!/usr/bin/env bash
# Generate ~/.ros/cyclonedds_wifi.xml for this machine.
#
#   ./make_dds_config.sh <peer-ip> [interface]
#
# <peer-ip>   the OTHER laptop's WiFi address
# [interface] optional; autodetected from the default route if omitted.
set -euo pipefail

PEER_IP="${1:-}"
if [[ -z "$PEER_IP" ]]; then
  echo "usage: $0 <peer-ip> [interface]" >&2
  echo "  run 'hostname -I' on the other laptop to find its address" >&2
  exit 1
fi

IFACE="${2:-$(ip route show default | awk '/default/ {print $5; exit}')}"
if [[ -z "$IFACE" ]]; then
  echo "could not autodetect an interface; pass one explicitly" >&2
  ip -br addr show >&2
  exit 1
fi

LOCAL_IP="$(ip -4 -br addr show "$IFACE" | awk '{print $3}' | cut -d/ -f1)"
if [[ -z "$LOCAL_IP" ]]; then
  echo "interface '$IFACE' has no IPv4 address" >&2
  exit 1
fi

if [[ "$LOCAL_IP" == "$PEER_IP" ]]; then
  echo "peer IP is this machine's own address; you want the OTHER laptop" >&2
  exit 1
fi

TEMPLATE="$(dirname "$(readlink -f "$0")")/../src/lidar_slam_bringup/config/cyclonedds_wifi.xml.template"
OUT="$HOME/.ros/cyclonedds_wifi.xml"
mkdir -p "$HOME/.ros"

sed -e "s|@@IFACE@@|$IFACE|g" \
    -e "s|@@LOCAL_IP@@|$LOCAL_IP|g" \
    -e "s|@@PEER_IP@@|$PEER_IP|g" \
    -e "s|@@LOGDIR@@|$HOME/.ros|g" \
    "$TEMPLATE" > "$OUT"

echo "wrote $OUT"
echo "  interface : $IFACE"
echo "  this host : $LOCAL_IP"
echo "  peer host : $PEER_IP"
echo
echo "Now source the environment in every shell that runs ROS on this machine:"
echo "  source $(dirname "$(readlink -f "$0")")/ros_network_env.sh"
