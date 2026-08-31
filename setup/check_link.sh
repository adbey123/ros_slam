#!/usr/bin/env bash
# Diagnose the laptop-to-laptop ROS2 link, in the order things actually break.
#
#   ./check_link.sh <peer-ip>
set -uo pipefail

PEER="${1:-}"
PASS=0; FAIL=0
ok()   { echo "  [ ok ] $*"; PASS=$((PASS+1)); }
bad()  { echo "  [FAIL] $*"; FAIL=$((FAIL+1)); }
note() { echo "         -> $*"; }

echo "=== 1. Environment ==="
if [[ -n "${ROS_DISTRO:-}" ]]; then ok "ROS_DISTRO=$ROS_DISTRO"
else bad "ROS not sourced"; note "source ~/lidar_slam_ws/setup/ros_network_env.sh"; fi

if [[ "${ROS_DOMAIN_ID:-}" == "42" ]]; then ok "ROS_DOMAIN_ID=42"
else bad "ROS_DOMAIN_ID='${ROS_DOMAIN_ID:-unset}' (expected 42)"
     note "both laptops must match exactly"; fi

if [[ "${RMW_IMPLEMENTATION:-}" == "rmw_cyclonedds_cpp" ]]; then ok "RMW=cyclonedds"
else bad "RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION:-unset}'"
     note "both laptops must use the same RMW"; fi

if [[ -f "${CYCLONEDDS_URI#file://}" ]] 2>/dev/null; then ok "CYCLONEDDS_URI resolves"
else bad "CYCLONEDDS_URI unset or missing"; note "./make_dds_config.sh <peer-ip>"; fi

echo
echo "=== 2. Network ==="
IFACE="$(ip route show default | awk '/default/ {print $5; exit}')"
LOCAL="$(ip -4 -br addr show "$IFACE" 2>/dev/null | awk '{print $3}' | cut -d/ -f1)"
[[ -n "$LOCAL" ]] && ok "this host: $LOCAL on $IFACE" || bad "no IPv4 on default route"

if [[ -z "$PEER" ]]; then
  echo "  [skip] no peer IP given; pass one to test reachability"
else
  if ping -c 2 -W 2 "$PEER" >/dev/null 2>&1; then ok "peer $PEER reachable"
  else bad "cannot ping $PEER"
       note "same WiFi network? some APs enable client isolation, which blocks"
       note "laptop-to-laptop traffic entirely -- check the AP settings"; fi

  if [[ -n "${CYCLONEDDS_URI:-}" ]] && grep -q "$PEER" "${CYCLONEDDS_URI#file://}" 2>/dev/null
  then ok "peer $PEER listed in the Cyclone config"
  else bad "peer $PEER not in the Cyclone peer list"
       note "./make_dds_config.sh $PEER"; fi
fi

if command -v ufw >/dev/null && sudo -n ufw status 2>/dev/null | grep -q "Status: active"; then
  bad "ufw is active"
  note "DDS uses ephemeral UDP ports; allow the subnet or disable ufw while testing"
else
  ok "no active ufw firewall detected"
fi

echo
echo "=== 3. ROS2 graph ==="
if ! command -v ros2 >/dev/null; then
  bad "ros2 not on PATH"
else
  NODES="$(timeout 8 ros2 node list 2>/dev/null | grep -v '^$' || true)"
  if [[ -n "$NODES" ]]; then ok "nodes visible:"; echo "$NODES" | sed 's/^/           /'
  else bad "no nodes visible"; note "start something on the other laptop first"; fi

  if timeout 8 ros2 topic list 2>/dev/null | grep -qx '/scan'; then
    ok "/scan is on the graph"
    echo "         measuring rate (5s)..."
    RATE="$(timeout 8 ros2 topic hz /scan 2>/dev/null | grep -m1 'average rate' || true)"
    [[ -n "$RATE" ]] && ok "$RATE" || bad "/scan advertised but no data arriving"
  else
    bad "/scan not visible"
    note "is lidar_laptop.launch.py running on the other machine?"
  fi
fi

echo
echo "=== 4. TF ==="
if command -v ros2 >/dev/null; then
  for pair in "base_link laser_frame" "odom base_link" "map odom"; do
    set -- $pair
    if timeout 6 ros2 run tf2_ros tf2_echo "$1" "$2" 2>&1 | grep -q "Translation"; then
      ok "$1 -> $2"
    else
      bad "$1 -> $2 missing"
    fi
  done
fi

echo
echo "=== $PASS passed, $FAIL failed ==="
[[ $FAIL -eq 0 ]]
